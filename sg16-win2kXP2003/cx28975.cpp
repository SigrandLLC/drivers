#include "common.h"

/* -----------------------------------------------------------------------------
 *    Wait for modem reply after issuing a command
 ------------------------------------------------------------------------------- */
bool
AdapterDesc::WaitForModemReply( void )
{
	Assert( KeGetCurrentIrql() <= DISPATCH_LEVEL ); //== PASSIVE_LEVEL );

	bool	Success=false;
	do
	{
		if( !NdisWaitEvent(&cx_intr, 3000) )
		{
			Debug( 9, this, "Modem didn't respond to command %02X",
				   cmdp->in_opcode );
			break;
		}

		NdisResetEvent( &cx_intr );
		Assert( (cmdp->out_ack & MACK_Status) != _ACK_NOT_COMPLETE );

		/* WaitForModemCompletes (); FIXME ! */
		Success=( (cmdp->out_ack & MACK_Status) == _ACK_PASS );
		if( !Success )
		{
			Debug( 9, this, "Modem command %02X failed (%02X)", cmdp->in_opcode,
				   cmdp->out_ack );
		}

		cmdp->out_ack=0;
	} while( False );
	return Success;
}

/* -----------------------------------------------------------------------------
 *    Issue modem command without waiting If both Data and Size are zero, one
 *    zero data byte is passed
 ------------------------------------------------------------------------------- */
void
AdapterDesc::SendModemMessage( BYTE Cmd, PCVOID Data, UINT Size )
{
	Assert( !!Data == !!Size );
	Assert( Size <= sizeof(cmdp->in_data) );
	Assert( KeGetCurrentIrql() <= DISPATCH_LEVEL );
	NdisResetEvent( &cx_intr );
	cmdp->in_dest=0xF0;
	cmdp->in_opcode=Cmd;
	cmdp->in_zero=0;
	if( Size )
	{
		Size--;
	}

	cmdp->in_length=BYTE( Size );
	cmdp->in_csum=BYTE( 0xF0 ^ Cmd ^ Size ^ 0xAA );

	BYTE	cksum=0;
	if( Data )
	{
		BYTE volatile	*mdp=cmdp->in_data;
		PCBYTE	hdp=PCBYTE( Data );
		for( UINT i=0; i <= Size; ++i )
		{
			cksum^=*hdp;
			*mdp++=*hdp++;	/* only 1 byte per cycle! */
		}
	} else
	{
		cmdp->in_data[0]=0;
	}

	cmdp->in_datasum=BYTE( cksum ^ 0xAA );
	cmdp->out_ack=_ACK_NOT_COMPLETE;
	cmdp->intr_8051=0xFE;
}

/* -----------------------------------------------------------------------------
 *
 ------------------------------------------------------------------------------- */
bool
AdapterDesc::DoModemCmd( BYTE Cmd, PCVOID Data, UINT Size,
						 ModemCompletionRoutine ComplRtn, DWORD_PTR RtnArg )
{
	if( ComplRtn )
	{
		Assert( !ModemComplRtn );
		ModemComplRtn=ComplRtn;
		ModemComplRtnArg=RtnArg;
	}

	SendModemMessage( Cmd, Data, Size );

	bool	Res=true;
	if( !ComplRtn )
	{
		Assert( KeGetCurrentIrql() <= DISPATCH_LEVEL );//== PASSIVE_LEVEL );
		DbgPrint("Kurrent irq level= %d\n",KeGetCurrentIrql());
		Res=WaitForModemReply();
	}

	return Res;
}

/* -----------------------------------------------------------------------------
 *  Link check function
  ------------------------------------------------------------------------------- */

ModemStates
GetCurModemState(cx28975_cmdarea *cmdp)
{
	BYTE const	AsmStatus=BYTE( cmdp->Status_1 & MST1_AsmStatus );
	if( AsmStatus == _ASM_STAT_SUCCESS )
	{
		Debug( 5, NULL, "Link established" );
		return ACTIVE;
	}
	return ACTIVATION;
}

VOID 
LinkCheckFunc(PVOID sysspiff1, PVOID AdapterContext,
                  PVOID sysspiff2, PVOID sysspiff3)
{
	Debug( 5, NULL, "Timer function" );
	AdapterDesc *ad= (AdapterDesc *)AdapterContext;
	ModemStates const	PrevState=ad->ModemState;

	ad->ModemState=GetCurModemState(ad->cmdp);
	if( ad->ModemState == PrevState )
		return;
	
	if( ad->ModemState == ACTIVE )
	{
		ad->HLDC->ResetStatusBits( ALL );
		ad->EnableReceiver();
		ad->Stat.attempts++;
		NdisGetCurrentSystemTime( &ad->Stat.last_time );
		/* !!!Unlock (); */
		if( !ad->ModemCfg.AlwaysConnected )
		{
			ad->Indicate( NDIS_STATUS_MEDIA_CONNECT );
		}
	}else if( PrevState == ACTIVE ){
		ad->DisableReceiver();
		/* !!!Unlock (); */
		if( !ad->ModemCfg.AlwaysConnected )
		{
			ad->Indicate( NDIS_STATUS_MEDIA_DISCONNECT );
		}
	}
}

/* -----------------------------------------------------------------------------
 *
 ------------------------------------------------------------------------------- */

void
AdapterDesc::cx28975_interrupt( void )
{
	if( cmdp->intr_host == 0xFE )
	{
		BYTE	Tmp;	/* To ensure register has been actually read */

		/* cmdp->intr_host = 0; */
		if( cmdp->out_ack & MACK_UnsolInt )
		{
			if( cmdp->Status_8 & MST8_AsmTransition ){
				Debug( 5, this, "Timer sheduled" );
				NdisMSetTimer(&LinkTimer,500);
			}

			cmdp->intr_host=0;		/* Remove interrupt message code */
			Tmp=cmdp->intr_host;	/* Clear interrupt */
			cmdp->out_ack=0;
		} else
		{
			cmdp->intr_host=0;		/* Remove interrupt message code */
			Tmp=cmdp->intr_host;	/* Clear interrupt */
			if( ModemComplRtn )
			{
				ModemCompletionRoutine const	Rtn=ModemComplRtn;
				ModemComplRtn=NULL;
				( this->*Rtn ) ( ModemComplRtnArg );
			} else
			{
				NdisSetEvent( &cx_intr );
			}
		}
	}
}

/* -----------------------------------------------------------------------------
 *
 ------------------------------------------------------------------------------- */
bool
AdapterDesc::download_firmware( PCBYTE const img, UINT const img_len )
{
	Debug( 0, this, "download_firmware:ENTER" );

	bool	Success=false;
	do
	{
		BYTE	cksum=0;
		for( UINT i=0; i < img_len; i++ )
		{
			cksum=BYTE( cksum + img[i] );
		}

		if( !DoModemCmd(_DSL_DOWNLOAD_START, PBYTE(&img_len), 4) )
		{
			break;
		}

		for( i=0; i < img_len; )
		{
			UINT const	PageSize=min( img_len - i, 75 );
			if( !DoModemCmd(_DSL_DOWNLOAD_DATA, img + i, PageSize) )
			{
				break;
			}

			i+=PageSize;
		}

		if( i < img_len )
		{
			break;
		}

		UINT const	t=( cksum ^ 0xff ) + 1;
		if( !DoModemCmd(_DSL_DOWNLOAD_END, PBYTE(&t), 1) )
		{
			break;
		}

		Success=true;
	} while( False );
	Debug( 0, this, "download_firmware:EXIT" );
	return Success;
}

/* -----------------------------------------------------------------------------
 *
 ------------------------------------------------------------------------------- */
bool
AdapterDesc::start_cx28975( PCVOID firmw_img, UINT firmw_len )
{
	bool	Success=false;
	do
	{
		cmdp->intr_host=0;
		NdisResetEvent( &cx_intr );

		/* reset chip set */
		HLDC->SetIntMask( EXT );
		HLDC->CRA=0;
		HLDC->ResetStatusBits( ALL );
		NdisStallExecution( 2 );
		HLDC->CRA=XRST;
		HLDC->CRB=RXDE;
		Debug( 5, this, "Programming HLDC" );
		if( HdlcCfg.crc16 )
		{
			HLDC->CRA|=CMOD;
		}

		if( HdlcCfg.fill_7e )
		{
			HLDC->CRA|=FMOD;
		}

		if( HdlcCfg.inv )
		{
			HLDC->CRA|=PMOD;
		}

		if( HdlcCfg.rburst )
		{
			HLDC->CRB|=RDBE;
		}

		if( HdlcCfg.wburst )
		{
			HLDC->CRB|=WTBE;
		}

		Debug( 5, this, "Waiting for modem reply after reset..." );
		Debug( 7, NULL,
			   "SG16: settings:\nRate=%d\nmaser=%d\ncfg=%d\nannex=%d\nmod=%d", ModemCfg.rate,
			   ModemCfg.master, ModemCfg.remcfg, ModemCfg.annex, ModemCfg.mod );
		NdisWaitEvent( &cx_intr, 10000 );
		if( (cmdp->out_ack & MACK_Status) != _ACK_BOOT_WAKE_UP )
		{
			Debug( 9, this, "No _ACK_BOOT_WAKE_UP responce" );
			break;
		}

		NdisResetEvent( &cx_intr );
		Debug( 5, this, "Downloading firmware" );
		if( !download_firmware(PCBYTE(firmw_img), firmw_len) ) break;
		Debug( 5, this, "Waiting for completion..." );
		NdisWaitEvent( &cx_intr, 10000 );
		if( (cmdp->out_ack & MACK_Status) != _ACK_OPER_WAKE_UP )
		{
			Debug( 9, this, "No _ACK_OPER_WAKE_UP response" );
			break;
		}

		NdisResetEvent( &cx_intr );
		Debug( 5, this, "Programming modem" );

		/* DSL_SYSTEM_ENABLE */
		BYTE	t=BYTE( 1 | (ModemCfg.master ? _DSL_HTUC : _DSL_HTUR) );
		if( !DoModemCmd(_DSL_SYSTEM_ENABLE, &t, 1) )
		{
			break;
		}

		/* DSL_SYSTEM_CONFIG */
		t=0x63;
		if( !DoModemCmd(_DSL_SYSTEM_CONFIG, &t, 1) )
		{
			break;
		}

		/* _DSL_MULTI_RATE_CONFIG dealing with rate */
		UINT	max_rate=0, min_rate=0;
		USHORT	tmp;
		if( ModemCfg.remcfg )
		{
			if( ModemCfg.master )
			{
				if( ModemCfg.annex == AnnexF )
					max_rate=MAX_REMCFGF_RATE;
				else if( ModemCfg.master )
					max_rate=MAX_REMCFGAB_RATE;
				min_rate=MIN_REMCFG_RATE;
			}
		} else
		{
			max_rate=MAX_RATE;
			min_rate=MIN_RATE;
		}


		if( max_rate )
		{
			tmp=WORD( (min_rate >> 3) & 0x3ff );
			ModemCfg.rate=( ModemCfg.rate < tmp ) ? tmp : ModemCfg.rate;
			tmp=WORD( (max_rate >> 3) & 0x3ff );
			ModemCfg.rate=( ModemCfg.rate > tmp ) ? tmp : ModemCfg.rate;
		} else
			ModemCfg.rate=192 >> 3;

		Debug( 7, NULL, "SG16: start, Rate=%d", ModemCfg.rate );

		BYTE	parm[12];
		NdisZeroMemory( parm, 12 );
		*( unsigned short * ) parm=WORD( (ModemCfg.rate >> 3) & 0x7f );
		Debug( 7, NULL, "SG16: start, parm[1,2]=%d", *( unsigned short * ) parm );
		parm[2]=parm[3]=parm[0];
		parm[5]=BYTE( ModemCfg.rate & 7 );
		parm[4]=parm[7]=1;
		parm[6]=0;
		if( !DoModemCmd(_DSL_MULTI_RATE_CONFIG, &parm, 8) )
		{
			break;
		}

		/* DSL_TRAINING_MODE */
		NdisZeroMemory( parm, 12 );
		if( ModemCfg.remcfg )
		{
			if( (ModemCfg.mod == 0x00 || ModemCfg.mod == 0x01) &&
				ModemCfg.master && ModemCfg.annex == AnnexF )
				parm[0]=BYTE( 0x02 | (ModemCfg.mod << 4) );
			else
				parm[0]=BYTE( 0x02 | 0x01 << 4 );
		} else
		parm[0]=BYTE( 0x02 | (ModemCfg.mod << 4) );
		parm[1]=0;
		if( !DoModemCmd(_DSL_TRAINING_MODE, parm, 2) )
		{
			break;
		}

		/* DSL_PREACTIVATION_CFG */
		NdisZeroMemory( parm, 12 );
		parm[0]=0x04;	/* pre-activation: G.hs */
		if( ModemCfg.remcfg ) parm[4]=0x00; /* HTU-C send Mode Select message */
		else
			parm[4]=0x04;	/* No remote configuration */
		parm[5]=0x01;		/* TPS-TC Config= Clear Channel */
		parm[6]=0x00;
		parm[7]=BYTE( ModemCfg.annex ); /* annex A,B,F */
		parm[8]=0xff;	/* i-bit mask */
		if( !DoModemCmd(_DSL_PREACTIVATION_CFG, parm, 12) )
		{
			break;
		}

		/* DSL_THRESHOLDS */
		NdisZeroMemory( parm, 12 );

		static const char	thresh[]={ +8, -4, -16, 40 };
		parm[0]=0x03;	/* dying gasp time - 3 frames */
		Assert( ModemCfg.mod < NumArrayElems(thresh) );
		parm[1]=thresh[ModemCfg.mod];
		parm[2]=0xff;	/* attenuation */
		parm[3]=0x04;	/* line probe NMR (+2 dB) */
		parm[4]=0x00;	/* reserved */
		parm[5]=0x00;
		if( !DoModemCmd(_DSL_THRESHOLDS, parm, 6) )
		{
			break;
		}

		/* DSL_FR_PCM_CONFIG */
		t=BYTE( ModemCfg.master ? 0x23 : 0x21 );
		if( !DoModemCmd(_DSL_FR_PCM_CONFIG, &t, 1) )
		{
			break;
		}

		/* DSL_INTR_HOST_MASK */
		t=0x02;
		if( !DoModemCmd(_DSL_INTR_HOST_MASK, &t, 1) )
		{
			break;
		}

		HLDC->SetIntMask( EXT );
		t=2;
		Verify( DoModemCmd(_DSL_CLEAR_ERROR_CTRS, &t, 1) );
		Debug( 5, this, "Activating modem" );
		if( DoModemCmd(_DSL_ACTIVATION, &t, 1) )
		{
			ModemState=ACTIVATION;
		}

		Success=true;
	} while( False );
	return Success;
}

/* -----------------------------------------------------------------------------
 *    Shutdown the modem
 ------------------------------------------------------------------------------- */
void
AdapterDesc::ShutdownModem( void )
{
	Assert( HLDC && HLDC->IMR == 0 );
	if( ModemComplRtn )
	{
		ModemCompletionRoutine const	Rtn=ModemComplRtn;
		ModemComplRtn=NULL;
		( this->*Rtn ) ( DWORD_PTR(-1) );
	}
}

/* -----------------------------------------------------------------------------
 *    Read modem stat counters
 ------------------------------------------------------------------------------- */
void
AdapterDesc::ReadModemStat( DWORD_PTR Stage )
{
	Assert( Stage <= 4 || Stage == DWORD_PTR(-1) );
	AssertIf( INT(Stage) > 0, (cmdp->out_ack & MACK_Status) != _ACK_NOT_COMPLETE );
	Debug( 4, this, "Read stat: stage %u", Stage );

	BYTE	Cmd=_DSL_NO_COMMAND;
	switch( Stage )
	{
	case 0:
		{
			Stage++;
			Cmd=_DSL_FAR_END_ATTEN;
			break;
		}

	case 1:
		{
			Stat.attenuat=cmdp->out_data[0];
			Stage++;
			Cmd=_DSL_NOISE_MARGIN;
			break;
		}

	case 2:
		{
			Stat.nmr=cmdp->out_data[0];
			Stage++;
			Cmd=_DSL_POWER_BACK_OFF_RESULT;
			break;
		}

	case 3:
		{
			Stat.tpbo=cmdp->out_data[0];
			Stat.rpbo=cmdp->out_data[1];
			Stage++;
			Cmd=_DSL_HDSL_PERF_ERR_CTRS;
			break;
		}

	case 4:
		{
			for( UINT i=0; i < 10; ++i )
			{
				PBYTE( &Stat.losw )[i]=cmdp->out_data[i];
			}

			Stat.status_1=cmdp->Status_1;
			Stat.status_3=cmdp->Status_3;
			Stage=0;
			cmdp->out_ack=_ACK_NOT_COMPLETE;
			ReadModemStatComplete();
			break;
		}

	case DWORD_PTR( -1 ):
		{
			ReadModemStatComplete( false );
			break;
		}
	}

	if( Cmd != _DSL_NO_COMMAND )
	{
		Assert( Stage );
		DoModemCmd( Cmd, NULL, 0, &AdapterDesc::ReadModemStat, Stage );
	}
}

/* -----------------------------------------------------------------------------
 *    Reset modem stat counters
 ------------------------------------------------------------------------------- */
void
AdapterDesc::ResetModemStat( DWORD_PTR Stage )
{
	Assert( Stage <= 2 || Stage == DWORD_PTR(-1) );
	AssertIf( INT(Stage) > 0, (cmdp->out_ack & MACK_Status) != _ACK_NOT_COMPLETE );
	Debug( 4, this, "Reset stat: stage %u", Stage );

	BYTE	t;
	switch( Stage )
	{
	case 0:
		{
			Stage++;
			t=_CLEAR_ALL_COUNTERS;
			break;
		}

	case 1:
		{
			Stage++;
			t=_CLEAR_SYSTEM_ERR_CTRS;
			break;
		}

	case 2:
		{
			cmdp->out_ack=_ACK_NOT_COMPLETE;
			ResetModemStatComplete();
			Stage=0;
			break;
		}

	case DWORD_PTR( -1 ):
		{
			ReadModemStatComplete( false );
			Stage=0;
			break;
		}
	}

	if( Stage )
	{
		DoModemCmd( _DSL_CLEAR_ERROR_CTRS, &t, 1, ResetModemStat, Stage );
	}
}
