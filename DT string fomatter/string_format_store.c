#include <stdio.h>
//#include <stdexcept>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
//#include "string_format.h"

#define MAX_DT_BUFF_LEN 1000
#define MAX_SEGMENT_LEN 2500


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

int find_multi_src_frames(struct node_DT_raw** list_HEAD, char* DT_buff_main, int len_main_buff, uint8_t PROTO);
int check_total_frame_len(char* DT_buff_main);
void removeSubstring(char *mainString, const char *substring);
int parse_byte_data(struct node_DT_raw* NODE, uint8_t* segment_buff, uint16_t len, uint8_t mFrameCnt);
void print_parsed_data(struct node_DT_raw* list_HEAD);
int push_DT_segment_to_list(struct node_DT_raw** list_HEAD, uint8_t* segment_buff, 
uint8_t total_char_in_segment, uint16_t mFrameBytes, uint8_t mFrameCnt);
void cleanup_DT_list(struct node_DT_raw* list_HEAD);
void cleanup_info_list(struct DT_info* list_HEAD);
int extract_dtc_from_raw_hex(struct node_DT_raw* DT_list_node, struct DT_info** info_list_HEAD);
void print_DT_info(struct DT_info* info_list_HEAD);

char DT_buff_main[MAX_DT_BUFF_LEN] = {0x00};
char DT_buff_main2[MAX_DT_BUFF_LEN+10] = {0x00};
char segment_buff[MAX_SEGMENT_LEN] = {0x00};

uint8_t PROTO;



//struct node_DT_raw* DT_raw_list = NULL;  //store our DTC here, everytime they arrive
//char single_frame_buf[250];

   /**Example 
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




int main()
{
    //malformed multi
    //sprintf(DT_buff_main ,"%s" , "01A-1CEBFF0B0104FF1503027E16-1CEBFF0B0203027E1703027E-1CEBFF0B031803027E220304-1CEBFF0B047E40020E00FFFF-18FECA0003FF00000000FFFF-01A-1CEBFF0B0104FF1503027E16-1CEBFF0B0203027E1703027E-1CEBFF0B031803027E220304----");
    
    //sprintf(DT_buff_main, "%s", "DAA");  //DAA
    
    //sprintf(DT_buff_main, "%s", "18FECA0003FF00000000");  //malformed single
    struct node_DT_raw* DT_raw_list = NULL;
     
    struct DT_info* DT_info_list = NULL;
    int tot_dtc_parsed = 0;

    //sprintf(DT_buff_main ,"%s" , "01A-1CEBFF0B0104FF1503027E16-1CEBFF0B0203027E1703027E-1CEBFF0B031803027E220304-1CEBFF0B047E40020E00FFFF-18FECA0003FF00000000FFFF-01A-1CEBFF170104FF1503027E16-1CEBFF170203027E1703027E-1CEBFF17031803027E220304-1CEBFF17047E40020E00FFFF----");
    //sprintf(DT_buff_main, "%s", "-18FECA0B04FF16030A03FFFF-18FECA0B04FF16030A03FFFF-022-18EBFF000141FF720000017C-18EBFF000200000117010001-18EBFF000312010001121100-18EBFF000401450500010C04-18EBFF0005000162040001FF-022-18EBFF000141FF720000017C-18EBFF000200000117010001-");
    //sprintf(DT_buff_main, "%s","00E-18EBFF0B0104FF1803057E17-18EBFF0B0203057E2A030E7E-00E-18EBFF170104FF1803057E17-18EBFF170203057E2A030E7E-03A-18EBFF000141FF2306004814-18EBFF00020600199DC20001-18EBFF000347200001A52400-18EBFF000407342000012324-18EBFF0005000147050002A4-18EBFF000624000155240001-18EBFF0007A0210001692200-18EBFF00081E9EC200012D24-18EBFF00091E9EFFFFFFFFFF-");
    //sprintf(DT_buff_main, "%s","00A-1CEBFF0B0104FF1703020815-1CEBFF0B0204020EFFFFFFFF-00A-1CEBFF0B0104FF1703020815-1CEBFF0B0204020EFFFFFFFF-18FECA0040FF69210011FFFF-18FECA0040FF69210011FFFF-18FECA0040FF69210011FFFF-");
    sprintf(DT_buff_main, "%s", "18FECA0003FF00000000FFFF-18FECA0103FF00000000FFFF-18FECA0BC0FF00000000FFFF-18FECA0303FF00000000FFFF-18FECA3D03FF00000000FFFF-18FECA1103FF00000000FFFF-18FECA1300FF0000007FFFFF-18FECA1900FF0000007FFFFF---18FECA2100FF0000007FFFFF-18FECA2A00FF0000007FFFFF-----18FECA7F00FF0000007FFFFF-18FECAE800FF0000007FFFFF-");
    if(DT_buff_main[0] != '-')  //add - here to keep things consistent
        sprintf(DT_buff_main2, "-%s-", DT_buff_main);

    else 
        sprintf(DT_buff_main2, "%s-", DT_buff_main);

    printf("recvd DT:%s\n\r", DT_buff_main2);
    
    int len = 0;
    len = check_total_frame_len(DT_buff_main2);  //basic checks

    if(len>0)
    {
        printf("\n\rLen main buff = %d",len);
        
        find_multi_src_frames(&DT_raw_list, DT_buff_main2, len, J1939);

        struct node_DT_raw* ptr = DT_raw_list;

        while(ptr!=NULL)
        {
            extract_dtc_from_raw_hex(ptr, &DT_info_list);
            ptr = ptr->next;
        }

        //printf("extracted %d DTCs",tot_dtc_parsed);
        print_parsed_data(DT_raw_list);
        print_DT_info(DT_info_list);

        cleanup_DT_list(DT_raw_list);
        cleanup_info_list(DT_info_list);        

    }
    return 0;
}


/*
*  0:NULL str
* -1:DAA
* -2:Malformed single frame
* -3:Malformed multiframe //ToDo
*/
int check_total_frame_len(char* DT_buff_main)  //Returns total frame len, and exceptions for garbage data
{
    if(DT_buff_main == NULL)
    {
        printf("\n\rNULL in total frame len , skip");
        return 0;
    }
        

    int len_main_buff = strlen(DT_buff_main);

    if(len_main_buff<24)  //always a multiple of 24 (excluding -)
    {
        if(strstr(DT_buff_main, "DAA"))
        {
            printf("\n\rno DT info recvd from uC");
            return -1;
        }
        else
        {
            printf("\n\rgarbage data recvd, skip");
            return -2;
        }
    }

    return len_main_buff;
}


//UTILS
void removeSubstring(char *mainString, const char *substring) {
    int substringLength = strlen(substring);
    int mainStringLength = strlen(mainString);
    int i, j, k;

    for (i = 0; i <= mainStringLength - substringLength; ++i) {
        // Check if the current substring matches starting from position i
        for (j = 0; j < substringLength; ++j) {
            if (mainString[i + j] != substring[j])
                break;
        }
        // If the substring matches, remove it
        if (j == substringLength) {
            // Shift characters after substring to the left
            for (k = i; k < mainStringLength - substringLength; ++k) {
                mainString[k] = mainString[k + substringLength];
            }
            // Null terminate the string to remove the leftover characters
            mainString[k] = '\0';
            mainStringLength -= substringLength;
            // Reset i to check for another occurrence of the substring from the current position
            i--;
        }
    }
}

int xtoi(uint8_t *p , uint16_t bytes)
{
    int k = 0;
    int val = 0;
    
    for(int i =0; i<(bytes*2); i++)
    {
        char c = *p;

        if(c >= '0' && c <= '9')
          val = (c - '0');
        else if (c >= 'A' && c <= 'F') 
          val = (10 + (c - 'A'));
        else if (c >= 'a' && c <= 'f')
          val = (10 + (c - 'a'));

        k = k*10 + val;
        p++;
    }

    return k;
}

int extract_dtc_from_raw_hex(struct node_DT_raw* DT_list_node, struct DT_info** info_list_HEAD)
{
    struct DT_info* ptr1 =  *info_list_HEAD;
    uint8_t tot_dtcs = 0;
    uint8_t nos_dtcs_in_buff = 0;
    uint8_t byte = 0;
   
    nos_dtcs_in_buff = ((strlen(DT_list_node->dataBuff)/2)-4)/4;  //data-lampbytes/(4 bytes per DTC)

    //struct DT_info* node = (struct DT_info*)malloc(sizeof(struct DT_info)); //new DT info node
    ptr1 = *info_list_HEAD;

    
    if(ptr1!= NULL)
        while(ptr1->next != NULL)
            ptr1 = ptr1->next; //iterate to last node

    //ptr1->next = node; //append node to last 
    

    //ptr1 = node;
    char* data_buf_ptr = (DT_list_node->dataBuff) + 4;
    for(int i = 0; i< nos_dtcs_in_buff; i++) //append remaining nodes
    {
        struct DT_info* next_node = (struct DT_info*)malloc(sizeof(struct DT_info)); //new DT info node

        if(ptr1!=NULL)
        {
            ptr1->next = next_node;
        }
        else
        {
            *info_list_HEAD = next_node;  //init list
        }

        next_node->src = xtoi(DT_list_node->src,1);
        //next_node->spn[0] = xtoi(data_buf_ptr,   1);  //8bit
        //next_node->spn[1] = xtoi(data_buf_ptr+2, 1);  //8bit
        //next_node->spn[2] = ((xtoi(data_buf_ptr+4, 1) >> 5) & 0x07); //3bit(H)
        memset(next_node->spn, 0x00, sizeof(next_node->spn));
        strncpy(next_node->spn,data_buf_ptr, 6);  //6 chars

        memset(next_node->elm_chars, 0x00, sizeof(next_node->elm_chars));
        strncpy(next_node->elm_chars,data_buf_ptr, 8);  //8 chars

        next_node->fmi =  (xtoi(data_buf_ptr+4, 1) & 0x1F);          //5bit(L)
        next_node->cm  =  ((xtoi(data_buf_ptr+6, 1) >> 7) & 0x01);     //1bit(H)
        next_node->oc  =  (xtoi(data_buf_ptr+6, 1) & 0x7F);     //1bit(H)

        data_buf_ptr+=8;

        ptr1 = next_node;
        
    }

}

void print_DT_info(struct DT_info* info_list_HEAD)
{
    uint8_t tot_faults = 0;
    struct DT_info* ptr = info_list_HEAD;

    while(ptr != NULL)
    {
        tot_faults++;

        printf("\n\r*********************");
        printf("\n\rDT info nos: %d", tot_faults);
        //printf("\n\rsrc:%.2X  spn1:%.2X  spn2:%.2X  spn3:%.2X  fmi:%.2X  cm:%.2X  oc:%.2X", 
        //ptr->src,ptr->spn[0],ptr->spn[1],ptr->spn[2],ptr->fmi,ptr->cm,ptr->oc);
        printf("\n\rsrc:%.2X, spn: %s, fmi:%.2X  cm:%.2X  oc:%.2X", 
        ptr->src,ptr->spn,ptr->fmi,ptr->cm,ptr->oc);
        printf("\n\r*********************\n\r");
        ptr = ptr->next;

    }

}

int parse_byte_data(struct node_DT_raw* NODE, uint8_t* segment_buff, uint16_t len, uint8_t mFrameCnt)
{
    char buff[500];
    memset(buff, 0x00, sizeof(buff));
    strncpy(buff, segment_buff, len);
    
    char header_buff[9] = {0x00};  //just the header
    char hyphen_header[10] = {0x00}; //header + hyphen
    char hyph_fc_header[12] = {0x00}; //hyphen + header + frame counter
    char* ptr;
    

    memset(header_buff, 0x00, sizeof(header_buff));
    memset(hyphen_header, 0x00, sizeof(hyphen_header));
    ptr = strstr(buff, "FECA");
    if(ptr != NULL)
    {
        strncpy(header_buff , ptr-2, 9);
    }
    else if(strstr(buff, "FECB") != NULL)
    {
        ptr = strstr(buff, "FECB");
        strncpy(header_buff , ptr-2, 9);
    }
    else if(strstr(buff, "EBFF") != NULL)
    {
        ptr = strstr(buff, "EBFF");
        strncpy(header_buff , ptr-2, 9);
    }
    else
    {
        printf("\n\rno valid frame header found\n\r");
        return -1;
    }
    snprintf(hyphen_header,10,"-%s",header_buff);  //-Header

    memset(NODE->src, 0x00, 3);
    NODE->src[0] = header_buff[6];
    NODE->src[1] = header_buff[7];

    if(mFrameCnt == 0)
        removeSubstring(buff, hyphen_header);

    else
    {
        for(int i = 1; i<=mFrameCnt; i++)
        {
            sprintf(hyph_fc_header, "%s%.2X",hyphen_header,i);
            removeSubstring(buff, hyph_fc_header);
        }
    }
    strncpy(NODE->dataBuff, buff, strlen(buff));
    NODE->lamp_status = xtoi(NODE->dataBuff, 2);
    return 0;

}

void print_parsed_data(struct node_DT_raw* list_HEAD)
{
    struct node_DT_raw* ptr1 = list_HEAD;

    if(ptr1 == NULL)
    {
        printf("\n\rno data found to print \n\r");
        return;
    }

    do
    {
        printf("\n\rstored data from src 0x%c%c : %s",ptr1->src[0],ptr1->src[1],ptr1->dataBuff);
        ptr1 = ptr1->next;

    } while (ptr1 != NULL);
    
}

int push_DT_segment_to_list(struct node_DT_raw** list_HEAD, uint8_t* segment_buff, 
uint8_t total_char_in_segment, uint16_t mFrameBytes, uint8_t mFrameCnt)
{
    struct node_DT_raw* node = (struct node_DT_raw*)malloc(sizeof(struct node_DT_raw));  
    
    struct node_DT_raw* ptr1 = *list_HEAD;

    if(ptr1 != NULL)
        while(ptr1->next != NULL)
            ptr1 = ptr1->next;

    parse_byte_data(node, segment_buff, total_char_in_segment, mFrameCnt);
    
    if(ptr1 != NULL)  //head is not null 
    {
        ptr1->next = node;
        node->next = NULL;
    }
    else
    {
        *list_HEAD = node;  //point list head to first node (first recursive call)
        node->next = NULL;
    }
    node->DT_frames = mFrameCnt;
    node->segment_bytes = mFrameBytes;

    return 1;

}

void cleanup_DT_list(struct node_DT_raw* list_HEAD)
{
    struct node_DT_raw* ptr2 = NULL;
    struct node_DT_raw* ptr1 = list_HEAD;
    uint8_t no_of_nodes = 0;

    if(list_HEAD == NULL)
        return;
    while(ptr1!= NULL)
    {
        ptr2 = ptr1;
        ptr1 = ptr1->next;
        free(ptr2);
        no_of_nodes++;
    }
    printf("\n\rcleanup DT list, %d nodes\n\r", no_of_nodes);
    
}

void cleanup_info_list(struct DT_info* list_HEAD)
{
    struct DT_info* ptr2 = NULL;
    struct DT_info* ptr1 = list_HEAD;
    uint8_t no_of_nodes = 0;

    if(list_HEAD == NULL)
        return;
    while(ptr1!= NULL)
    {
        ptr2 = ptr1;
        ptr1 = ptr1->next;
        free(ptr2);
        no_of_nodes++;
    }
    printf("\n\rcleanup info list, %d nodes\n\r", no_of_nodes);
    
}


/*ret:
*  -1 : error in parsing
*   0 : successfully parsed everything
*/

int find_multi_src_frames(struct node_DT_raw** list_HEAD, char* segment_ptr, int len_main_buff, uint8_t PROTO)   //take an input DT and seperate it into frames (single and multi), based on src address.
{
    char* ptr1 = NULL;
    char* ptr2 = NULL;
    static uint8_t nos_multi_frame = 0;  //these need to retain values at every recursive call
    static uint8_t nos_single_frame = 0;
    

    uint16_t multi_frame_bytes = 0;
    uint16_t total_char_in_segment = 0;
    uint8_t multi_frame_cnt = 0;
    uint8_t multi_frame_detected;
    
    //static struct node_DT_raw* list_HEAD = NULL;


    ptr1 = segment_ptr;
    //leaf conditions

    switch(PROTO)
    {
        case J1939:
            if(len_main_buff<24)  
            {
                printf("\n\rreached end of DT, exiting");
                return -1;
            }
            

            if(strstr(ptr1,"FECA")==NULL &&  strstr(ptr1,"FECB")==NULL && strstr(ptr1,"ECFF")==NULL && strstr(ptr1,"EBFF")==NULL)  //no valid header found
            {
                printf("\n\rno valid header found, exiting");
                return -1;
            }
        break;

        /*case UDS_BB6:
            if(len_main_buff<24)  
            {
                printf("\n\rreached end of DT, exiting");
                return list_HEAD;
            }

            if(strstr(ptr1,"18DAF1")==NULL)  //no valid header found
            {
                printf("\n\rno valid header found, exiting");
                return list_HEAD;
            }
        break;*/

    }
    
        
    switch(PROTO)
    {
        case J1939:
            if((*(ptr1 + 3) == '-' ||  *(ptr1 + 4) == '-')  &&  (strstr(ptr1, "EBFF")) != NULL)  //we are assuming that the frame always starts with the byte count or -
            {
                //handle an extra '-' at the beginning
                if(*ptr1 == '-')
                    ptr1++;

                nos_multi_frame ++;
                multi_frame_detected = 1;
                multi_frame_bytes = strtol(ptr1, &ptr2, 16);
                multi_frame_cnt = multi_frame_bytes%7==0?(multi_frame_bytes/7):(multi_frame_bytes/7)+1;
                printf("\n\rdetected multiframe nos:%d multiframe,byte cnt:%d , frame cnt = %d",nos_multi_frame,multi_frame_bytes, multi_frame_cnt);

                //all hex chars(2 per byte) + headers(8 char per frame)+ frame_counter(2char per frame)  +'-'(1 per frame) + filler bytes(for now take them also)
                total_char_in_segment = ((multi_frame_bytes*2)+ (8*multi_frame_cnt)+(2*multi_frame_cnt) + multi_frame_cnt + (((multi_frame_cnt*7)-multi_frame_bytes))*2);
                
            }
            else if((strstr(ptr1, "FECA")) != NULL  ||   (strstr(ptr1, "FECB")) != NULL)       //first frame is not multi frame. store this single frame and move ahead
            {
                nos_single_frame++;
                multi_frame_detected = 0;
                printf("\n\rdetected single frame nos:%d",nos_single_frame);
                ptr2 = ptr1;  //+1; //skip the '-'
                total_char_in_segment = 25; 

            }
            else
            {
                printf("\n\r Garbage data \n\r");
                return -1;
            }
        break;


        /*case UDS_BB6: //Todo
            if(strstr("18DAF1", ptr1) != NULL)
            {
                if(xtoi(ptr1+8,1) == 0x10)  //multi frame header
                {
                    printf("\n\rdetected UDS multiframe");
                    nos_multi_frame ++;
                    multi_frame_detected = 1;

                }

                else
                {
                    printf("\n\rsingle frame header");
                }
                    

            }
            else{
                printf("\n\r garbage data\n\r");
            } 
        break;*/
    }



    

    memset(segment_buff , 0x00, sizeof(segment_buff));
    strncpy(segment_buff ,ptr2, total_char_in_segment);

    if(*(ptr2+total_char_in_segment) != '-') //something is wrong
    {
        printf("\n\rmalformed frames");
        printf("\n\rlast char = %c", *(ptr2+total_char_in_segment));
        return -1;
    }
    else
    {   
        int i = 1;
        while(*(ptr2+total_char_in_segment+i) == '-')  //check for multiple '-'
            i++;
            
        ptr2+=total_char_in_segment + (i-1);  //move ptr2 to start of remaining segment
    }
    
    push_DT_segment_to_list(list_HEAD, segment_buff, total_char_in_segment, multi_frame_bytes , multi_frame_cnt);
    //printf("total 1st seg = %d, total remaining = %d", total_char_in_segment, strlen(ptr2) );
    find_multi_src_frames(list_HEAD, ptr2 , strlen(ptr2), PROTO);

    return 0;
    
}
