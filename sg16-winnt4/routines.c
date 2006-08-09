
#include "precomp.h"

static int      download_firmware( PSBNI16_ADAPTER, PUCHAR, UINT );


void
cx28975_interrupt( PSBNI16_ADAPTER Adapter )
{
        volatile struct cx28975_cmdarea  *p = Adapter->cmdp;
        UCHAR  t;

        if( p->intr_host != 0xfe )
                return;

        p->intr_host = 0;
        t = p->intr_host;

        if( p->out_ack & 0x80 ) {
                if( *((UCHAR *)p + 0x3c7) & 2 ) {
                        if( Adapter->state != ACTIVE
                            &&  (*((PUCHAR)p + 0x3c0) & 0xc0) == 0x40 )
                                activate( Adapter );
                        else if( Adapter->state == ACTIVE
                                 &&  (*((PUCHAR)p + 0x3c0) & 0xc0) != 0x40 )
                                deactivate( Adapter );
                }

//              p->intr_host = 0;
//              t = p->intr_host;
                p->out_ack = 0;
        } else {
//              p->intr_host = 0;
//              t = p->intr_host;

                if( Adapter->rdstat == 0 )
                        NdisSetEvent( &Adapter->cx_intr );
                else
                        do_rdstat( Adapter );
        }
}

/* -------------------------------------------------------------------------- */

int
start_cx28975( PSBNI16_ADAPTER Adapter, PUCHAR firmw_img, UINT firmw_len )
{
        static char  thresh[] = { +8, -4, -16, -40 };

        volatile struct cx28975_cmdarea  *p = Adapter->cmdp;
        struct cx28975_cfg  cfg = Adapter->cfg;
        UCHAR  t, parm[ 12 ];

        p->intr_host = 0;
        t = p->intr_host;

        /* reset chip set */
        Adapter->regs->IMR = EXT;
        Adapter->regs->CR  = 0;
        Adapter->regs->SR  = 0xff;
        NdisMSleep( 2 );
        Adapter->regs->CR = XRST;
        if( cfg.crc16 )         Adapter->regs->CR |= CMOD;
        if( cfg.fill_7e )       Adapter->regs->CR |= FMOD;
        if( cfg.inv )           Adapter->regs->CR |= PMOD;

        Adapter->regs->CRB |= RXDE;
        if( cfg.rburst )        Adapter->regs->CRB |= RDBE;
        if( cfg.wburst )        Adapter->regs->CRB |= WTBE;

        NdisWaitEvent( &Adapter->cx_intr, 10000 );
        if( (p->out_ack & 0x1f) != _ACK_BOOT_WAKE_UP )
                return  -1;

//DbgPrint( "BOOT_WAKE_UP\n" );
        NdisResetEvent( &Adapter->cx_intr );
        if( download_firmware( Adapter, firmw_img, firmw_len ) )
                return  -1;

        NdisWaitEvent( &Adapter->cx_intr, 10000 );
        if( (p->out_ack & 0x1f) != _ACK_OPER_WAKE_UP )
                return  -1;
//DbgPrint( "OPER_WAKE_UP\n" );

        NdisResetEvent( &Adapter->cx_intr );
        t = cfg.master ? 1 : 9;
        if( issue_cx28975_cmd( Adapter, _DSL_SYSTEM_ENABLE, &t, 1 ) )
                return  -1;

        t = 0x63;
        if( issue_cx28975_cmd( Adapter, _DSL_SYSTEM_CONFIG, &t, 1 ) )
                return  -1;

        *(PUSHORT)parm = cfg.rate >> 6;
        parm[2] = parm[3] = parm[0];
        parm[5] = (cfg.rate >> 3) & 7;
        parm[4] = parm[7] = 1;
        parm[6] = 0;
        if( issue_cx28975_cmd( Adapter, _DSL_MULTI_RATE_CONFIG, parm, 8 ) )
                return  -1;

        parm[0] = 0x02 | (cfg.mod << 4);
        parm[1] = 0;
        if( issue_cx28975_cmd( Adapter, _DSL_TRAINING_MODE, parm, 2 ) )
                return  -1;

        NdisZeroMemory( parm, 12 );
        parm[0] = 0x04;         /* pre-activation: G.hs */
        parm[4] = 0x04;         /* no remote configuration */
        parm[7] = 0x01;         /* annex A (default) */
        parm[8] = 0xff;         /* i-bit mask (all bits) */
        if( issue_cx28975_cmd( Adapter, _DSL_PREACTIVATION_CFG, parm, 12 ) )
                return  -1;

        parm[0] = 0x03;         /* dying gasp time - 3 frames */
        parm[1] = thresh[ cfg.mod ];
        parm[2] = 0xff;         /* attenuation */
        parm[3] = 0x04;         /* line probe NMR (+2 dB) */
        parm[4] = 0x00;         /* reserved */
        parm[5] = 0x00;
        if( issue_cx28975_cmd( Adapter, _DSL_THRESHOLDS, parm, 6 ) )
                return  -1;

        t = cfg.master ? 0x23 : 0x21;
        if( issue_cx28975_cmd( Adapter, _DSL_FR_PCM_CONFIG, &t, 1 ) )
                return  -1;

        t = 0x02;
        if( issue_cx28975_cmd( Adapter, _DSL_INTR_HOST_MASK, &t, 1 ) )
                return  -1;

        Adapter->regs->IMR = EXT;
        t = 2;
        issue_cx28975_cmd( Adapter, _DSL_CLEAR_ERROR_CTRS, &t, 1 );
        if( issue_cx28975_cmd( Adapter, _DSL_ACTIVATION, &t, 1 ) == 0 )
                Adapter->state = ACTIVATION;
//DbgPrint( "Activation started\n" );

        return  0;
}


static int
download_firmware( PSBNI16_ADAPTER Adapter, PUCHAR img, UINT img_len )
{
        UINT    t, i;
        UCHAR   cksum = 0;

        for( i = 0;  i < img_len;  ++i )
                cksum += img[i];

        t = img_len;
        if( issue_cx28975_cmd( Adapter, _DSL_DOWNLOAD_START, (PUCHAR) &t, 4 ) )
                return  -1;

        for( i = 0;  img_len >= 75;  i += 75, img_len -= 75 )
                if( issue_cx28975_cmd( Adapter, _DSL_DOWNLOAD_DATA, img + i, 75 ) )
                        return  -1;

        if( img_len
            &&  issue_cx28975_cmd( Adapter, _DSL_DOWNLOAD_DATA, img + i, (UCHAR) img_len ) )
                return  -1;

        t = (cksum ^ 0xff) + 1;
        if( issue_cx28975_cmd( Adapter, _DSL_DOWNLOAD_END, (PUCHAR) &t, 1 ) )
                return  -1;

        return  0;
}


int
issue_cx28975_cmd( PSBNI16_ADAPTER Adapter, UCHAR cmd, PUCHAR data, UCHAR size )
{
        volatile struct cx28975_cmdarea  *p = Adapter->cmdp;
        volatile PUCHAR  databuf = (volatile PUCHAR) p->in_data;

        int  i;

        UCHAR  cksum = 0;

        p->in_dest      = 0xf0;
        p->in_opcode    = cmd;
        p->in_zero      = 0;
        p->in_length    = --size;
        p->in_csum      = 0xf0 ^ cmd ^ size ^ 0xaa;

        for( i = 0;  i <= size;  ++i )
                cksum ^= *data,
                *databuf++ = *data++;   /* only 1 byte per cycle! */

        p->in_datasum   = cksum ^ 0xaa;
        p->out_ack      = _ACK_NOT_COMPLETE;
        p->intr_8051    = 0xfe;

        if( !NdisWaitEvent( &Adapter->cx_intr, 3000 ) )
                return  -1;
        NdisResetEvent( &Adapter->cx_intr );

        while( p->out_ack == _ACK_NOT_COMPLETE )
                ;                                       /* FIXME ! */

        if( (p->out_ack & 0x1f) == _ACK_PASS ) {
                p->out_ack = 0;
                return  0;
        } else {
                p->out_ack = 0;
                return  -1;
        }
}


void
activate( PSBNI16_ADAPTER Adapter )
{
        Adapter->regs->SR   = 0xff;             /* clear it! */
        Adapter->regs->CTDR = Adapter->regs->LTDR = Adapter->regs->CRDR = Adapter->regs->LRDR = 0;

        Adapter->head_tdesc = Adapter->head_rdesc = 0;
        alloc_rx_buffers( Adapter );

        Adapter->regs->CRB &= ~RXDE;
        Adapter->regs->IMR = EXT | RXS | TXS | OFL | UFL;
        Adapter->regs->CR |= TXEN | RXEN;

        Adapter->state = ACTIVE;
        ++Adapter->in_stats.attempts;
        NdisGetCurrentSystemTime( &Adapter->in_stats.last_time );

        NdisReleaseSpinLock( &Adapter->Lock );
        NdisMIndicateStatus( Adapter->mport,
                             NDIS_STATUS_MEDIA_CONNECT, NULL, 0 );
        NdisMIndicateStatusComplete( Adapter->mport );
        NdisAcquireSpinLock( &Adapter->Lock );
}


void
deactivate( PSBNI16_ADAPTER Adapter )
{
        Adapter->regs->CR  &= ~(RXEN | TXEN);
        Adapter->regs->CRB |= RXDE;
        Adapter->regs->IMR  = EXT;
        Adapter->regs->CTDR = Adapter->regs->LTDR;
        Adapter->regs->CRDR = Adapter->regs->LRDR;
        Adapter->state = ACTIVATION;

        drop_queues( Adapter );

        NdisReleaseSpinLock( &Adapter->Lock );
        NdisMIndicateStatus( Adapter->mport,
                             NDIS_STATUS_MEDIA_DISCONNECT, NULL, 0 );
        NdisMIndicateStatusComplete( Adapter->mport );
        NdisAcquireSpinLock( &Adapter->Lock );
}


void
start_rdstat( PSBNI16_ADAPTER Adapter )
{
        volatile struct cx28975_cmdarea  *p = Adapter->cmdp;

        NdisAcquireSpinLock( &Adapter->Lock );

        p->in_dest      = 0xf0;
        p->in_opcode    = _DSL_FAR_END_ATTEN;
        p->in_zero      = 0;
        p->in_length    = 0;
        p->in_csum      = 0xf0 ^ _DSL_FAR_END_ATTEN ^ 0xaa;
        p->in_data[0]   = 0;
        p->in_datasum   = 0xaa;

        p->out_ack      = _ACK_NOT_COMPLETE;
        p->intr_8051    = 0xfe;

        Adapter->rdstat = 1;
        NdisReleaseSpinLock( &Adapter->Lock );
}


void
do_rdstat( PSBNI16_ADAPTER Adapter )
{
        volatile struct cx28975_cmdarea  *p = Adapter->cmdp;
        UINT   i;
        UCHAR  op;

        while( p->out_ack == _ACK_NOT_COMPLETE )
                ;

        switch( Adapter->rdstat ) {
        case  1 :
                Adapter->in_stats.attenuat = p->out_data[ 0 ];
                Adapter->rdstat = 2;
                op = _DSL_NOISE_MARGIN;
                break;

        case  2 :
                Adapter->in_stats.nmr = p->out_data[ 0 ];
                Adapter->rdstat = 3;
                op = _DSL_POWER_BACK_OFF_RESULT;
                break;

        case  3 :
                Adapter->in_stats.tpbo = p->out_data[ 0 ];
                Adapter->in_stats.rpbo = p->out_data[ 1 ];
                Adapter->rdstat = 4;
                op = _DSL_HDSL_PERF_ERR_CTRS;
                break;

        case  4 :
                Adapter->rdstat = 0;
                for( i = 0;  i < 10;  ++i )
                        ((PUCHAR) &Adapter->in_stats.losw)[i] = p->out_data[i];
                Adapter->in_stats.status_1 = ((PUCHAR) p)[ 0x3c0 ];
                Adapter->in_stats.status_3 = ((PUCHAR) p)[ 0x3c2 ];
                p->out_ack = 0;
                return;
        }

        p->in_dest      = 0xf0;
        p->in_opcode    = op;
        p->in_zero      = 0;
        p->in_length    = 0;
        p->in_csum      = 0xf0 ^ op ^ 0xaa;

        p->out_ack      = _ACK_NOT_COMPLETE;
        p->intr_8051    = 0xfe;
}

