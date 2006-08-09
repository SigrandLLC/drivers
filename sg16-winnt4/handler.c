
#include "precomp.h"

static void     resume_tx( PSBNI16_ADAPTER );
static void     start_xmit_frames( PSBNI16_ADAPTER );
static BOOLEAN  encap_frame( PSBNI16_ADAPTER );
static void     copy_packet_data( PSBNI16_ADAPTER, PNDIS_PACKET );
static void     free_sent_buffers( PSBNI16_ADAPTER );
static void     free_packet( PSBNI16_ADAPTER, PNDIS_PACKET );
static void     indicate_frames( PSBNI16_ADAPTER );


VOID
SB16Isr( OUT PBOOLEAN    InterruptRecognized,
         OUT PBOOLEAN    QueueDPC,
         IN NDIS_HANDLE  Context
        )
{
        PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) Context;
        UCHAR  status = Adapter->regs->SR;

        *QueueDPC = FALSE;

        if( status == 0 ) {
                *InterruptRecognized = FALSE;
                return;
        }

        *InterruptRecognized = TRUE;
        if( status & CRC )
                ++Adapter->in_stats.crc_errs,
                ++Adapter->CrcErrors,
                Adapter->regs->SR = CRC;

        if( status & OFL )
                ++Adapter->in_stats.ofl_errs,
                Adapter->regs->SR = OFL;

        if( Adapter->regs->SR ) {
                Adapter->regs->IMR = 0;
                *QueueDPC = TRUE;
                return;
        }
}


VOID
SB16HandleInterrupt( IN NDIS_HANDLE  Context )
{
        PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) Context;
        UCHAR  status;

        NdisAcquireSpinLock( &Adapter->Lock );
        status = Adapter->regs->SR;

        if( status & EXT )
                cx28975_interrupt( Adapter ),
                Adapter->regs->SR = EXT;

        if( status & UFL )
                resume_tx( Adapter ),
                Adapter->regs->SR = UFL,
                ++Adapter->XmitBad,
                ++Adapter->in_stats.ufl_errs;

        if( status & RXS )
                Adapter->regs->SR = RXS,
                indicate_frames( Adapter ),
                alloc_rx_buffers( Adapter );

        if( status & TXS )
                Adapter->regs->SR = TXS,
                free_sent_buffers( Adapter );

        Adapter->regs->IMR = EXT | (Adapter->state == ACTIVE ?
                                        RXS | TXS | OFL | UFL : 0 );

        NdisReleaseSpinLock( &Adapter->Lock );
}


VOID
SB16MultipleSend( NDIS_HANDLE context, PPNDIS_PACKET p, UINT count )
{
        PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) context;
        UINT  i;

        NdisAcquireSpinLock( &Adapter->Lock );

        for( i = 0;  i < count;  ++i ) {
                NDIS_SET_PACKET_STATUS( *p, NDIS_STATUS_PENDING );
                if( Adapter->xw_head == NULL )
                        Adapter->xw_head = *p;
                if( Adapter->xw_tail != NULL )
                        PKT_LINK( Adapter->xw_tail ) = *p;
                PKT_LINK( *p )   = NULL;
                COPY_FLAG( *p )  = 0;
                Adapter->xw_tail = *p++;
//DbgPrint( "enq tx pkt\n" );
        }

        if( Adapter->state == ACTIVE )
                start_xmit_frames( Adapter );
        NdisReleaseSpinLock( &Adapter->Lock );
}


/*
 * Look for a first descriptor of a next packet, and write it's number
 * into CTDR. Then enable the transmitter.
 */

static void
resume_tx( PSBNI16_ADAPTER  Adapter )
{
        UINT    cur_tbd = Adapter->regs->CTDR;

        while( cur_tbd != Adapter->regs->LTDR
                && (Adapter->tbd[ cur_tbd++ ].length & LAST_FRAG) == 0 )
                ;
        Adapter->regs->CTDR = (UCHAR)cur_tbd;
        Adapter->regs->CR |= TXEN;
}


static void
start_xmit_frames( PSBNI16_ADAPTER  Adapter )
{
        /*
         * Check if we have any free descriptor(s) and free space in
         * our transmit queue.
         */
        while( Adapter->tail_xq != ((Adapter->head_xq - 1) & (XQLEN - 1))
                &&  Adapter->xw_head != NULL  &&  encap_frame( Adapter ) )
                ;
}


__inline void
dequeue_xpkt( PSBNI16_ADAPTER Adapter, PNDIS_PACKET pkt )
{
        Adapter->xw_head = PKT_LINK( pkt );
        if( Adapter->xw_tail == pkt )
                Adapter->xw_tail = NULL;
}


/* MUST be called at spin lock acquired */

static BOOLEAN
encap_frame( PSBNI16_ADAPTER  Adapter )
{
        NDIS_PHYSICAL_ADDRESS_UNIT  phys_desc[ 16 ];

        PNDIS_PACKET  pkt = Adapter->xw_head;
        PNDIS_BUFFER  buf;
        UINT  i, chunks, length, cur_tbd;
        INT   busy_tbd;

        cur_tbd  = Adapter->regs->LTDR & 0x7f;
        busy_tbd = cur_tbd - Adapter->head_tdesc + 1;
        if( busy_tbd < 0 )
                busy_tbd = -busy_tbd;

        NdisQueryPacket( pkt, &chunks, NULL, &buf, &length );

        if( COPY_FLAG( pkt ) ) {
                if( !Adapter->cont_pg_busy  &&  busy_tbd < 127 ) {
                        dequeue_xpkt( Adapter, pkt );
                        copy_packet_data( Adapter, pkt );
                        Adapter->tbd[ cur_tbd ].address = Adapter->cont_pg_phys;
                        Adapter->tbd[ cur_tbd ].length  = length;

                        ++cur_tbd;
                        cur_tbd &= 0x7f;
                        goto  finalize;
                } else
                        return  FALSE;
        }

        if( chunks > (128 - (UINT)busy_tbd) )
                return  FALSE;

        dequeue_xpkt( Adapter, pkt );

        do {
                chunks = 0;
                NdisMStartBufferPhysicalMapping( Adapter->mport, buf,
                        Adapter->free_map_reg, TRUE, phys_desc, &chunks );
                ++Adapter->free_map_reg;
                Adapter->free_map_reg &= (MAP_REGS - 1);

                for( i = 0;  i < chunks;  ++i ) {
                        /* check the buffer length */
                        if( phys_desc[ i ].Length == 0 )
                                continue;

                        if( phys_desc[ i ].Length < 5 ) {
                                /*
                                 * a special case: we have to roll back
                                 * and copy all the data
                                 */
                                PNDIS_BUFFER  t;

                                NdisQueryPacket( pkt, NULL, NULL, &t, NULL );
                                while( t ) {
                                        NdisMCompleteBufferPhysicalMapping(
                                                        Adapter->mport, t,
                                                        Adapter->old_map_reg );
                                        ++Adapter->old_map_reg;
                                        Adapter->old_map_reg &= (MAP_REGS - 1);
                                        NdisGetNextBuffer( t, &t );
                                }

                                COPY_FLAG( pkt ) = 1;
                                if( !Adapter->cont_pg_busy ) {
                                        cur_tbd  = Adapter->regs->LTDR & 0x7f;
                                        copy_packet_data( Adapter, pkt );
                                        Adapter->tbd[ cur_tbd ].address = Adapter->cont_pg_phys;
                                        Adapter->tbd[ cur_tbd ].length  = length;

                                        ++cur_tbd;
                                        cur_tbd &= 0x7f;
                                        goto  finalize;
                                } else {
                                        /* append it back to the queue */
                                        PKT_LINK( pkt ) = Adapter->xw_head;
                                        Adapter->xw_head = pkt;
                                        if( !Adapter->xw_tail )
                                                Adapter->xw_tail = pkt;
                                        return  FALSE;
                                }
                        }

                        Adapter->tbd[ cur_tbd ].address = NdisGetPhysicalAddressLow( phys_desc[ i ].PhysicalAddress );
                        Adapter->tbd[ cur_tbd ].length  = phys_desc[ i ].Length;

                        ++cur_tbd;
                        cur_tbd &= 0x7f;
                }

                /* Flush the current buffer because it could be cached */
                NdisFlushBuffer( buf, TRUE );
                NdisGetNextBuffer( buf, &buf );
        } while( buf );

finalize:
        if(Adapter->tbd[(cur_tbd-1)&0x7f].length<60)
         Adapter->tbd[ (cur_tbd - 1) & 0x7f ].length=60;

        Adapter->tbd[ (cur_tbd - 1) & 0x7f ].length |= LAST_FRAG;

        Adapter->xq[ Adapter->tail_xq++ ] = pkt;
        Adapter->tail_xq &= (XQLEN - 1);

        Adapter->regs->LTDR = (UCHAR)cur_tbd;
        ++Adapter->in_stats.sent_pkts;
        ++Adapter->XmitGood;
//DbgPrint( "deq tx pkt\n" );
        return  TRUE;
}



static void
copy_packet_data( PSBNI16_ADAPTER Adapter, PNDIS_PACKET pkt )
{
        PNDIS_BUFFER  buf;
        UINT  i, chunks, length, cur_tbd;
        PUCHAR  cur_pos = Adapter->cont_pg_virt;

        NdisQueryPacket( pkt, NULL, NULL, &buf, NULL );

        while( buf ) {
                UINT    cur_len;
                PVOID   addr;
//                 NdisQueryBufferSafe(buf,&addr,&cur_len,16);
                NdisQueryBuffer( buf, &addr, &cur_len );
                NdisMoveMemory( cur_pos, addr, cur_len );
                cur_pos += cur_len;
                NdisGetNextBuffer( buf, &buf );
        }

        Adapter->cont_pg_busy = TRUE;
        ++Adapter->in_stats.copied;
//DbgPrint( "claim cont pg\n" );
}


static void
free_sent_buffers( PSBNI16_ADAPTER  Adapter )
{
        UINT  cur_tbd;

        cur_tbd = Adapter->regs->CTDR;

        while( Adapter->head_tdesc != cur_tbd ) {
                /*
                 * Be careful! one element in xq may corresponds to
                 * multiple descriptors.
                 */
                if( Adapter->tbd[ Adapter->head_tdesc ].length & LAST_FRAG ) {
                        PNDIS_PACKET  pkt = Adapter->xq[ Adapter->head_xq++ ];
                        Adapter->head_xq &= (XQLEN - 1);
                        free_packet( Adapter, pkt );
                }

                Adapter->tbd[ Adapter->head_tdesc ].length = 0;
                Adapter->head_tdesc = (Adapter->head_tdesc + 1) & 0x7f;
        }

        start_xmit_frames( Adapter );
}

/* DON'T use free_sent_buffers to drop the queue! */


static void
free_packet( PSBNI16_ADAPTER Adapter, PNDIS_PACKET pkt )
{
        if( COPY_FLAG( pkt ) )
//DbgPrint( "free cont pg\n" ),
                Adapter->cont_pg_busy = FALSE;
        else {
                PNDIS_BUFFER  buf;

                NdisQueryPacket( pkt, NULL, NULL, &buf, NULL );
                while( buf ) {

                        NdisMCompleteBufferPhysicalMapping( Adapter->mport, buf,
                                                Adapter->old_map_reg );
                        ++Adapter->old_map_reg;
                        Adapter->old_map_reg &= (MAP_REGS - 1);
                        NdisGetNextBuffer( buf, &buf );
                }
        }

        NdisReleaseSpinLock( &Adapter->Lock );
        NdisMSendComplete( Adapter->mport, pkt, NDIS_STATUS_SUCCESS );
        NdisAcquireSpinLock( &Adapter->Lock );
}


void
alloc_rx_buffers( PSBNI16_ADAPTER Adapter )
{
        struct sw_rbd  *p = Adapter->swr_head;
        UINT    cur_rbd = Adapter->regs->LRDR & 0x7f;

//DbgPrint( "alloc_rx_buffers enter\n" );
        while( Adapter->tail_rq != ((Adapter->head_rq - 1) & (RQLEN - 1)) ) {

                if( (p = Adapter->swr_head) == NULL )
                        return;

                if( (Adapter->swr_head = p->link) == NULL )
                        Adapter->swr_tail = NULL;

                Adapter->rq[ Adapter->tail_rq++ ] = p;
                Adapter->tail_rq &= (RQLEN - 1);

                Adapter->rbd[ cur_rbd ].address = p->phys_addr;
                Adapter->rbd[ cur_rbd ].length  = 0;
                Adapter->regs->LRDR = cur_rbd = (cur_rbd + 1) & 0x7f;
        }
//DbgPrint( "alloc_rx_buffers exit\n" );
}


static void
indicate_frames( PSBNI16_ADAPTER Adapter )
{
        NDIS_PACKET     *pkt_array[ 8 ];
        UINT            pkt_count = 0, i, len;
        UINT            cur_rbd = Adapter->regs->CRDR & 0x7f;

//DbgPrint( "indicate_frames enter\n" );
        while( Adapter->head_rdesc != cur_rbd ) {
                struct sw_rbd  *p = Adapter->rq[ Adapter->head_rq++ ];
                Adapter->head_rq &= (RQLEN - 1);
                --Adapter->free_rb;
                len=Adapter->rbd[ Adapter->head_rdesc ].length & 0x7ff;
                //if(len<60)len=60;
                NdisAdjustBufferLength( p->buf_ptr,len);
                NDIS_SET_PACKET_STATUS( p->pkt_ptr, Adapter->free_rb > 3
                        ?  NDIS_STATUS_SUCCESS  :  NDIS_STATUS_RESOURCES );

                pkt_array[ pkt_count++ ] = p->pkt_ptr;
                ++Adapter->in_stats.rcvd_pkts;
                ++Adapter->RcvGood;

                Adapter->head_rdesc = (Adapter->head_rdesc + 1) & 0x7f;
        }

        if( pkt_count ) {
//DbgPrint( "indicate %d\n", pkt_count );
                NdisReleaseSpinLock( &Adapter->Lock );
                NdisMIndicateReceivePacket( Adapter->mport, pkt_array, pkt_count );
                NdisAcquireSpinLock( &Adapter->Lock );
        }

        for( i = 0;  i < pkt_count;  ++i )
                if( NDIS_GET_PACKET_STATUS( pkt_array[ i ] )
                    != NDIS_STATUS_PENDING )
                        reclaim( Adapter, SW_RBD_PTR( pkt_array[ i ] ) );
//DbgPrint( "indicate_frames exit\n" );
}


void
drop_queues( PSBNI16_ADAPTER Adapter )
{
        PNDIS_PACKET  p;

        while( Adapter->head_rq != Adapter->tail_rq )
                reclaim( Adapter, Adapter->rq[ Adapter->head_rq++ ] ),
                Adapter->head_rq &= (RQLEN - 1);

        while( Adapter->head_xq != Adapter->tail_xq ) {
                p = Adapter->xq[ Adapter->head_xq++ ];
                Adapter->head_xq &= (XQLEN - 1);
                free_packet( Adapter, p );
        }

        while( Adapter->xw_head ) {
                p = Adapter->xw_head;
                Adapter->xw_head = PKT_LINK( p );

                NdisReleaseSpinLock( &Adapter->Lock );
                NdisMSendComplete( Adapter->mport, p, NDIS_STATUS_FAILURE );
                NdisAcquireSpinLock( &Adapter->Lock );
        }

        Adapter->xw_tail = NULL;
}


VOID
SB16GetReturnedPackets( NDIS_HANDLE context, PNDIS_PACKET pkt )
{
        PSBNI16_ADAPTER  Adapter = (PSBNI16_ADAPTER) context;

//DbgPrint( "GetReturnedPackets enter\n" );
        NdisAcquireSpinLock( &Adapter->Lock );
        reclaim( Adapter, SW_RBD_PTR( pkt ) );
        alloc_rx_buffers( Adapter );
        NdisReleaseSpinLock( &Adapter->Lock );
//DbgPrint( "GetReturnedPackets exit\n" );
}


void
reclaim( PSBNI16_ADAPTER Adapter, struct sw_rbd *p )
{
        NdisAdjustBufferLength( p->buf_ptr, SBNI16_MAX_FRAME );

        p->link = NULL;
        if( Adapter->swr_tail )
                Adapter->swr_tail->link = p;
        Adapter->swr_tail = p;
        if( !Adapter->swr_head )
                Adapter->swr_head = p;

        ++Adapter->free_rb;
}

