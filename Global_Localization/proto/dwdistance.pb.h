/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.8 at Wed Apr 26 04:42:02 2017. */

#ifndef PB_DWDISTANCE_PB_H_INCLUDED
#define PB_DWDISTANCE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _DwDistance {
    uint32_t send_id;
    uint32_t recv_id;
    double dist;
    bool beacon;
/* @@protoc_insertion_point(struct:DwDistance) */
} DwDistance;

/* Default values for struct fields */

/* Initializer values for message structs */
#define DwDistance_init_default                  {0, 0, 0, 0}
#define DwDistance_init_zero                     {0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define DwDistance_send_id_tag                   2
#define DwDistance_recv_id_tag                   3
#define DwDistance_dist_tag                      4
#define DwDistance_beacon_tag                    5

/* Struct field encoding specification for nanopb */
extern const pb_field_t DwDistance_fields[5];

/* Maximum encoded size of messages (where known) */
#define DwDistance_size                          23

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define DWDISTANCE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
