#include <stdio.h>
//#include <stdexcept>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "string_format.h"

#define MAX_DT_BUFF_LEN 1000
#define MAX_SEGMENT_LEN 2500





void removeSubstring(char *mainString, const char *substring);
int parse_byte_data(struct node_DT_raw* NODE, uint8_t* segment_buff, uint16_t len, uint8_t mFrameCnt);
int push_DT_segment_to_list(struct node_DT_raw** list_HEAD, uint8_t* segment_buff, 
uint8_t total_char_in_segment, uint16_t mFrameBytes, uint8_t mFrameCnt);
int extract_dtc_from_raw_hex(struct node_DT_raw* DT_list_node, struct DT_info** info_list_HEAD);


char DT_buff_main[MAX_DT_BUFF_LEN] = {0x00};


uint8_t PROTO;







int main()
{
    //in this list, raw string data (without headers) will be stored, based on src.
    struct node_DT_raw* DT_raw_list = NULL;
     
    //In this list, parsed J1939 DTCs will be stored 
    struct DT_info* DT_info_list = NULL;

    //here, formated string to send to FC41D will be stored.

    int tot_dtc_parsed = 0;

    char* ELM_buff[400] = {0x00};

    /*DT_buff_main 
    *   Is the input buffer received from renesas. remove the "" and "DT:" key and copy the string data  
    *
    */

    //sprintf(DT_buff_main ,"%s" , "01A-1CEBFF0B0104FF1503027E16-1CEBFF0B0203027E1703027E-1CEBFF0B031803027E220304-1CEBFF0B047E40020E00FFFF-18FECA0003FF00000000FFFF-01A-1CEBFF170104FF1503027E16-1CEBFF170203027E1703027E-1CEBFF17031803027E220304-1CEBFF17047E40020E00FFFF----");
    //sprintf(DT_buff_main, "%s", "-18FECA0B04FF16030A03FFFF-18FECA0B04FF16030A03FFFF-022-18EBFF000141FF720000017C-18EBFF000200000117010001-18EBFF000312010001121100-18EBFF000401450500010C04-18EBFF0005000162040001FF-022-18EBFF000141FF720000017C-18EBFF000200000117010001-");
    //sprintf(DT_buff_main, "%s","00E-18EBFF0B0104FF1803057E17-18EBFF0B0203057E2A030E7E-00E-18EBFF170104FF1803057E17-18EBFF170203057E2A030E7E-03A-18EBFF000141FF2306004814-18EBFF00020600199DC20001-18EBFF000347200001A52400-18EBFF000407342000012324-18EBFF0005000147050002A4-18EBFF000624000155240001-18EBFF0007A0210001692200-18EBFF00081E9EC200012D24-18EBFF00091E9EFFFFFFFFFF-");
    //sprintf(DT_buff_main, "%s","00A-1CEBFF0B0104FF1703020815-1CEBFF0B0204020EFFFFFFFF-00A-1CEBFF0B0104FF1703020815-1CEBFF0B0204020EFFFFFFFF-18FECA0040FF69210011FFFF-18FECA0040FF69210011FFFF-18FECA0040FF69210011FFFF-");
    sprintf(DT_buff_main, "%s", "18FECA0003FF00000000FFFF-18FECA0103FF00000000FFFF-18FECA0BC0FF00000000FFFF-18FECA0303FF00000000FFFF-18FECA3D03FF00000000FFFF-18FECA1103FF00000000FFFF-18FECA1300FF0000007FFFFF-18FECA1900FF0000007FFFFF---18FECA2100FF0000007FFFFF-18FECA2A00FF0000007FFFFF-----18FECA7F00FF0000007FFFFF-18FECAE800FF0000007FFFFF-");
    

    //Run this block everytime DT is recieved
    int len = 0;
    len = check_total_frame_len(DT_buff_main);  //basic checks, adds '-' at beginning if not present

    if(len>0)  //check if null
    {
        printf("\n\rLen main buff = %d",len);
        
        find_multi_src_frames(&DT_raw_list, DT_buff_main, len, J1939);  //list ptr, buffer, length, protocol type

        struct node_DT_raw* ptr = DT_raw_list;

        while(ptr!=NULL)
        {
            extract_dtc_from_raw_hex(ptr, &DT_info_list);
            ptr = ptr->next;
        }

        //printf("extracted %d DTCs",tot_dtc_parsed);
        print_parsed_raw_data(DT_raw_list);  //show raw char info
        print_DT_info(DT_info_list);     //show parsed DTC info

        format_ELM_buff(DT_info_list, ELM_buff);

        //IMPORTANT!!, run cleanup DT_list everytime after find_multi_src_frames populates DT_raw_list 
        // and extract_dtc_from_raw_hex populates DT_info_list. each object should be cleaned before loop exits.
        cleanup_DT_list(DT_raw_list);    
        cleanup_info_list(DT_info_list);        

    }
    else
        return -1;

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
    char* ptr = DT_buff_main; //passing pointer by reference
    if(ptr == NULL)
    {
        printf("\n\rNULL in total frame len , skip");
        return 0;
    }
        
    
    int len_main_buff = strlen(ptr);

    if(len_main_buff<24)  //always a multiple of 24 (excluding -)
    {
        if(strstr(ptr, "DAA"))
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


    char* temp = (char*)calloc(MAX_DT_BUFF_LEN+10, sizeof(char));
    if(ptr[0] != '-')  //add - here to keep things consistent
    {  
        sprintf(temp, "-%s-", ptr);
    }
    else 
    {
        sprintf(temp, "%s-", ptr);
    }

    memset(ptr, 0x00, sizeof(ptr));
    strcpy(ptr, temp ); //, sizeof(temp));
    free(temp);

    printf("recvd DT:%s\n\r", ptr);

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
        // Note: Changed the below arrays of zero dtc strings and garbage dtc strings
        //var zeroDTCStrings = [ "18FECA0B43FF54000202FFFF", "18FECA0040BF00000000FFFF" ];
        char* buff_temp[9] = {0x00};
        strncpy(buff_temp, data_buf_ptr, 8);

        if(strstr(buff_temp,"54000202") != NULL ||  strstr(buff_temp,"00000000") != NULL  
        ||  strstr(buff_temp,"FFFFFFFF") != NULL  ||  strstr(buff_temp,"0000007F") != NULL)  //skip this zero/garbage DTC
        {
            data_buf_ptr+=8;
            continue;
        }

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
void format_ELM_buff(struct DT_info* info_list_HEAD, char* ELM_buff)
{
    if(ELM_buff == NULL)
        return;

    uint8_t nos_DTC = 0;

    char* temp_bytes_buff[400] = {0x00};
    char* bytes[5] = {0x00};

    memset(ELM_buff, 0x00, sizeof(ELM_buff));
    sprintf(ELM_buff, "$SDG&S=0&e=0,0,4,");

    while(info_list_HEAD!=NULL)
    {
        strcat(temp_bytes_buff,",");
        strcat(temp_bytes_buff, info_list_HEAD->elm_chars);

        nos_DTC++;
        info_list_HEAD = info_list_HEAD->next;

    }

    if(nos_DTC == 0);
    {
        memset(ELM_buff, 0x00, sizeof(ELM_buff));
        printf("\n\ryay, no DTC detected :)\n\r");
    }
    sprintf(bytes, "%d", nos_DTC);
    strcat(ELM_buff, bytes);
    strcat(ELM_buff, temp_bytes_buff);

    printf("\n\r\n\rResponse to ELM: %s\n\r\n\r", ELM_buff);

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

void print_parsed_raw_data(struct node_DT_raw* list_HEAD)
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
    
    char segment_buff[MAX_SEGMENT_LEN] = {0x00};


    ptr1 = segment_ptr;
    //leaf conditions (end recursion)

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

    
    //parsing scheme. Chop the raw chars based on multiframe/singleframe and store in DT_raw_list
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
                ptr2 = ptr1; 
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
