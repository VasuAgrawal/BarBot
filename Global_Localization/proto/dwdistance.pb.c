/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.8 at Tue Apr 11 03:28:49 2017. */

#include "dwdistance.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t DwDistance_fields[6] = {
    PB_FIELD(  1, MESSAGE , SINGULAR, STATIC  , FIRST, DwDistance, time, time, &google_protobuf_Timestamp_fields),
    PB_FIELD(  2, UINT32  , SINGULAR, STATIC  , OTHER, DwDistance, send_id, time, 0),
    PB_FIELD(  3, UINT32  , SINGULAR, STATIC  , OTHER, DwDistance, recv_id, send_id, 0),
    PB_FIELD(  4, DOUBLE  , SINGULAR, STATIC  , OTHER, DwDistance, dist, recv_id, 0),
    PB_FIELD(  5, BOOL    , SINGULAR, STATIC  , OTHER, DwDistance, beacon, dist, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(DwDistance, time) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_DwDistance)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(DwDistance, time) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_DwDistance)
#endif


/* On some platforms (such as AVR), double is really float.
 * These are not directly supported by nanopb, but see example_avr_double.
 * To get rid of this error, remove any double fields from your .proto.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)

/* @@protoc_insertion_point(eof) */
