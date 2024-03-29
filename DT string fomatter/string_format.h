#include <stdint.h>

struct node_DT_raw
{
    struct node_DT_raw* next;
    char src[3];
    uint16_t segment_bytes;
    uint16_t nos_data_bytes;  //headerless
    uint8_t DT_frames;
    char dataBuff[200];    //headerless bytes
    uint16_t lamp_status;  //adding here as only two bytes for each src are same
};

struct DT_info
{
    struct DT_info* next;
    uint8_t src;
    char spn[7];        //uint32?
    uint8_t fmi;           
    uint8_t oc;            //J1939
    uint8_t cm;            //J1939
    char elm_chars[9];  //4 bytes to send to ELM
};

enum PROTO_TYPE{
    J1939 = 1,
    UDS_BB6 = 2
};




   /**Example  J1939
   * Single-frame string: 18FECA0043FFB804038AFFFF
   * 		Parts:
   * 			18FECA00
   * 				43	byte 1		// lamp status
   * 				FF	byte 2		// lamp status
   * 				B8	byte 3		// spn 8 bits
   * 				04	byte 4		// spn 8 bits
   * 				03	byte 5		// spn 3 bits + fmi 5bits
   * 				8A	byte 6		// cm 1 bit + oc 7 bits
   * 				FFFF			// garbage
   * 
   * Multi-frame string: 	00A18EBFF00017FFF640002017818EBFF0002020502FFFFFFFF
   * 00A 18EBFF00 01 7FFF 64000201 78 18EBFF00 02 020502 FFFFFFFF
   * 0,0,4,2,64000201,78020502
   * 		Parts:
   * 			00A
   * 			18EBFF00
   * 				01	byte 1		// Frame no 1
   * 				7F  byte 2		// lamp status
   * 				FF	byte 3		// lamp status
   * 				64	byte 4		// spn1 8 bits
   * 				00	byte 5		// spn1 8 bits
   * 				02	byte 6		// spn1 3 bits + fmi 5 bits
   * 				01	byte 7		// cm 1 bit + oc 7 bits
   * 				78	byte 8		// spn2 8bits
   * 			18EBFF00			// skip
   * 				02	byte 9		// Frame no 2
   * 				02	byte 10 	// spn2 8bits
   * 				05	byte 11		// spn2 3 bits + fmi 5bits
   * 				02	byte 12		// cm 1 bit + oc 7 bits
   * 			FFFFFFFF			//garbage
   */


//Public functions
int find_multi_src_frames(struct node_DT_raw** list_HEAD, char* DT_buff_main, int len_main_buff, uint8_t PROTO);
int check_total_frame_len(char* DT_buff_main);
void print_parsed_raw_data(struct node_DT_raw* list_HEAD);
void print_DT_info(struct DT_info* info_list_HEAD);
void cleanup_DT_list(struct node_DT_raw* list_HEAD);
void cleanup_info_list(struct DT_info* list_HEAD);
void format_ELM_buff(struct DT_info* info_list_HEAD, char* ELM_buff);