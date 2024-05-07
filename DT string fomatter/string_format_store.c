#include <stdio.h>
//#include <stdexcept>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "string_format.h"

#define MAX_DT_BUFF_LEN 1000
#define MAX_SEGMENT_LEN 2500



//if you want to store the lists for doing other stuff, call these seperately
int find_multi_src_frames(struct node_DT_raw** list_HEAD, char* DT_buff_main, int len_main_buff, uint8_t PROTO);
int check_total_frame_len(char* DT_buff_main);
void print_parsed_raw_data(struct node_DT_raw* list_HEAD);
void print_DT_info(struct DT_info* info_list_HEAD);
void cleanup_DT_list(struct node_DT_raw* list_HEAD);
void cleanup_info_list(struct DT_info* list_HEAD);
void format_ELD_buff(struct DT_info* info_list_HEAD, char* ELM_buff, uint8_t PROTO);


//Internal
void removeSubstring(char *mainString, const char *substring);
int parse_byte_data(struct node_DT_raw* NODE, uint8_t* segment_buff, uint16_t len, uint8_t mFrameCnt, uint8_t PROTO);
int push_DT_segment_to_list(struct node_DT_raw** list_HEAD, uint8_t* segment_buff, 
uint8_t total_char_in_segment, uint16_t mFrameBytes, uint8_t mFrameCnt, uint8_t PROTO);
int extract_dtc_from_raw_hex(struct node_DT_raw* DT_list_node, struct DT_info** info_list_HEAD, uint8_t PROTO);


char DT_buff_main[MAX_DT_BUFF_LEN] = {0x00};

//uint8_t PROTO;

int main()
{
    //in this list, raw string data (without headers) will be stored, based on src.
    struct node_DT_raw* DT_raw_list = NULL;
     
    //In this list, parsed J1939 DTCs will be stored 
    struct DT_info* DT_info_list = NULL;

    //here, formated string to send to FC41D will be stored.

    int tot_dtc_parsed = 0;

    //char ELD_buff[400] = {0x00};

    char* buff_ptr;

    /*DT_buff_main 
    *   Is the input buffer received from renesas. remove the "" and "DT:" key and copy the string data  
    *
    */

    //sprintf(DT_buff_main ,"%s" , "01A-1CEBFF0B0104FF1503027E16-1CEBFF0B0203027E1703027E-1CEBFF0B031803027E220304-1CEBFF0B047E40020E00FFFF-18FECA0003FF00000000FFFF-01A-1CEBFF170104FF1503027E16-1CEBFF170203027E1703027E-1CEBFF17031803027E220304-1CEBFF17047E40020E00FFFF----");
    //sprintf(DT_buff_main, "%s", "-18FECA0B04FF16030A03FFFF-18FECA0B04FF16030A03FFFF-022-18EBFF000141FF720000017C-18EBFF000200000117010001-18EBFF000312010001121100-18EBFF000401450500010C04-18EBFF0005000162040001FF-022-18EBFF000141FF720000017C-18EBFF000200000117010001-");
    //sprintf(DT_buff_main, "%s","00E-18EBFF0B0104FF1803057E17-18EBFF0B0203057E2A030E7E-00E-18EBFF170104FF1803057E17-18EBFF170203057E2A030E7E-03A-18EBFF000141FF2306004814-18EBFF00020600199DC20001-18EBFF000347200001A52400-18EBFF000407342000012324-18EBFF0005000147050002A4-18EBFF000624000155240001-18EBFF0007A0210001692200-18EBFF00081E9EC200012D24-18EBFF00091E9EFFFFFFFFFF-");
    //sprintf(DT_buff_main, "%s","00A-1CEBFF0B0104FF1703020815-1CEBFF0B0204020EFFFFFFFF-00A-1CEBFF0B0104FF1703020815-1CEBFF0B0204020EFFFFFFFF-18FECA0040FF69210011FFFF-18FECA0040FF69210011FFFF-18FECA0040FF69210011FFFF-");
    //sprintf(DT_buff_main, "%s", "18FECA0003FF00000000FFFF-18FECA0103FF00000000FFFF-18FECA0BC0FF00000000FFFF-18FECA0303FF00000000FFFF-18FECA3D03FF00000000FFFF-18FECA1103FF00000000FFFF-18FECA1300FF0000007FFFFF-18FECA1900FF0000007FFFFF---18FECA2100FF0000007FFFFF-18FECA2A00FF0000007FFFFF-----18FECA7F00FF0000007FFFFF-18FECAE800FF0000007FFFFF-");
    //sprintf(DT_buff_main, "%s","-18DAF100100B590239DB040E-18DAF10021287F020E28040E-18DAF101035902FFFFFFFFFF-18DAF13D10135902FF43150F-18DAF13D2128431510284315-18DAF13D220028870E1F28FF-");
    //sprintf(DT_buff_main, "%s","18DAF100102F5902FFC28200-18DAF1002128C29D00282169-18DAF10022002822A1006814-18DAF100230300AF00730028-18DAF100242BAC00AF2BA300-18DAF10025AF26E200AF26E5-18DAF10026002F22690028AA");
    
    //sprintf(DT_buff_main, "%s","0:4305010004011:01130101008700");

    sprintf(DT_buff_main, "%s","8CC2039EE406-8CC2039EE406-88C011C2101546B10801B73107F10301FA0747B1-88C009C2110801F50304F701-");

    buff_ptr = parse_dtc(eld_dt_j1708, DT_buff_main);
    printf("\n\rparse DTC: %s\n\r", buff_ptr);

    if(buff_ptr != NULL)
        free(buff_ptr);  //do this to prevent memory leaks, beacuse ptr is stack variable


    return 0;
}



/**
 * 
 * Top level API for ELD. returns the ELD buffer with formatted response, otherwise returns a NULL.
 * 
*/
char* parse_dtc(uint8_t prt, char* dtc_string) 
{
    struct node_DT_raw* DT_raw_list = NULL;
     
    //In this list, parsed J1939 DTCs will be stored 
    struct DT_info* DT_info_list = NULL;

    //here, formated string to send to FC41D will be stored.

    int tot_dtc_parsed = 0;

    char* ELD_buff = NULL;

    if(prt < 1)
    {
        printf("\n\r returned NULL from parse_dtc 1");
        return NULL;
    }
        

    if(dtc_string == NULL)
    {
        printf("\n\r returned NULL from parse_dtc 2");
        return NULL;
    }

    int len = 0;
    len = check_total_frame_len(dtc_string);  //basic checks, adds '-' at beginning if not present

    if(len>0)  //check if null
    {
        printf("\n\rLen main buff = %d",len);
        
        find_multi_src_frames(&DT_raw_list, dtc_string, len, prt);  //list ptr, buffer, length, protocol type
        printf("\n\rreached here 1");

        struct node_DT_raw* ptr = DT_raw_list;

        

        while(ptr!=NULL)
        {
            printf("\n\r in while");
            extract_dtc_from_raw_hex(ptr, &DT_info_list, prt);
            ptr = ptr->next;
        }

        printf("extracted %d DTCs",tot_dtc_parsed);
        print_parsed_raw_data(DT_raw_list);  //show raw char info
        
        print_DT_info(DT_info_list);     //show parsed DTC info

        ELD_buff = calloc(sizeof(char)* ELD_BUFF_LEN, sizeof(char)); 

        if(ELD_buff == NULL)
        {
            printf("\n\rmalloc failed, loc 1\n\r");
            return NULL;
        }       

        format_ELD_buff(DT_info_list, ELD_buff, prt);

        printf("\n\r reached here 3\n\r");

        //IMPORTANT!!, run cleanup DT_list everytime after find_multi_src_frames populates DT_raw_list 
        // and extract_dtc_from_raw_hex populates DT_info_list. each object should be cleaned before loop exits.*/
        cleanup_DT_list(DT_raw_list);    
        cleanup_info_list(DT_info_list);  

        uint16_t eld_len = strlen(ELD_buff);

        if(eld_len == 0)
        {
            printf("\n\r no DTC to send to ELD\n\r");
            free(ELD_buff);
            return NULL;
        }
        else
        {
            printf("\n\r returned NULL from parse DTC 3");
            return ELD_buff; //max LEN is 400
        }



    }
    else
        return NULL;
    
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

    if(len_main_buff<16)  //always a multiple of 16 (excluding -)
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

    if(temp == NULL)
    {
        printf("\n\r calloc failed 1\n\r");
        return 0;
    }

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

        k = k*16 + val;
        p++;
    }

    return k;
}

int extract_dtc_from_raw_hex(struct node_DT_raw* DT_list_node, struct DT_info** info_list_HEAD, uint8_t PROTO)
{
    struct DT_info* ptr1 =  *info_list_HEAD;
    uint8_t tot_dtcs = 0;
    uint8_t nos_dtcs_in_buff = 0;
    uint8_t byte = 0;
   
    char Pcode;
    uint8_t two_bits = 0;
    uint8_t nibble = 0;
    uint8_t nibbles[4];
    char* data_buf_ptr;

    switch (PROTO)
    {
    case eld_dt_j1939:
        nos_dtcs_in_buff = ((strlen(DT_list_node->dataBuff)/2)-4)/4;  //data-lampbytes/(4 bytes per DTC)
        data_buf_ptr= (DT_list_node->dataBuff) + 4;
    break;

    case eld_dt_uds3:
    case eld_dt_uds_bb6:
        nos_dtcs_in_buff = ((strlen(DT_list_node->dataBuff)/2)-2)/4;
        data_buf_ptr= (DT_list_node->dataBuff) + 2;  //first byte is status
    break;

    case eld_dt_obd2:
        nos_dtcs_in_buff = (strlen(DT_list_node->dataBuff)/2)/2;   //2 bytes per dtc, no lamp bytes
        data_buf_ptr = (DT_list_node->dataBuff);
    break;

    case eld_dt_j1708:
        nos_dtcs_in_buff = (strlen(DT_list_node->dataBuff)/2)/3;   //3 bytes per dtc, no lamp bytes
        data_buf_ptr = (DT_list_node->dataBuff);
    break;
    
    default:
        break;
    }
    

    //struct DT_info* node = (struct DT_info*)malloc(sizeof(struct DT_info)); //new DT info node
    ptr1 = *info_list_HEAD;

    
    if(ptr1!= NULL)
        while(ptr1->next != NULL)
            ptr1 = ptr1->next; //iterate to last node

    //ptr1->next = node; //append node to last 
    

    //ptr1 = node;

    
        
    
    for(int i = 0; i< nos_dtcs_in_buff; i++) //append remaining nodes
    {
        // Note: Changed the below arrays of zero dtc strings and garbage dtc strings
        //var zeroDTCStrings = [ "18FECA0B43FF54000202FFFF", "18FECA0040BF00000000FFFF" ];
        

        if(PROTO == eld_dt_j1939)
        {
            char buff_temp[9] = {0x00};
            strncpy(buff_temp, data_buf_ptr, 8);

            if(strstr(buff_temp,"54000202") != NULL ||  strstr(buff_temp,"00000000") != NULL  
            ||  strstr(buff_temp,"FFFFFFFF") != NULL  ||  strstr(buff_temp,"0000007F") != NULL)  //skip this zero/garbage DTC
            {
                data_buf_ptr+=8;  //4 bytes DTC
                continue;
            }
        }
        else
        {
            char buff_temp[9] = {0x00};
            strncpy(buff_temp, data_buf_ptr, 4);

            if(strstr(buff_temp,"0000") != NULL ||  strstr(buff_temp,"FFFF") != NULL)
            {
                data_buf_ptr+=4;  //2 bytes DTC
                continue;
            }
        }

        

        struct DT_info* next_node = (struct DT_info*)calloc(sizeof(struct DT_info), sizeof(char)); //new DT info node
        if(next_node == NULL)
        {
            printf("\n\r malloc failed loc 3");
            //return 0;
        }

        if(ptr1!=NULL)
        {
            ptr1->next = next_node;
        }
        else
        {
            *info_list_HEAD = next_node;  //init list
        }

        switch(PROTO)
        {
            case eld_dt_j1939:
                

                next_node->src = xtoi(DT_list_node->src,1);
                memset(next_node->spn, 0x00, sizeof(next_node->spn));
                strncpy(next_node->spn,data_buf_ptr, 6);  //6 chars
                memset(next_node->eld_chars, 0x00, sizeof(next_node->eld_chars));
                strncpy(next_node->eld_chars,data_buf_ptr, 8);  //8 chars
                next_node->fmi =  (xtoi(data_buf_ptr+4, 1) & 0x1F);          //5bit(L)
                next_node->cm  =  ((xtoi(data_buf_ptr+6, 1) >> 7) & 0x01);     //1bit(H)
                next_node->oc  =  (xtoi(data_buf_ptr+6, 1) & 0x7F);     //1bit(H)

                data_buf_ptr+=8;
            break;

            case eld_dt_uds3:
            case eld_dt_uds_bb6:
            case eld_dt_obd2:
                

                for(int i =0; i<4 ; i++)
                {
                    if(*(data_buf_ptr+i) >= '0' && *(data_buf_ptr+i) <='9')
                        nibbles[i] = *(data_buf_ptr+i) - '0';  //atoi

                    else if(*(data_buf_ptr+i) >= 'A' && *(data_buf_ptr+i) <= 'F')
                        nibbles[i] = 10 + (*(data_buf_ptr+i) - 'A');  //atoi
                }

                two_bits = (nibbles[0]>>2) & 0x03;

                if(two_bits == 0)
                    Pcode = 'P';
                else if(two_bits == 1)
                    Pcode = 'C';
                else if(two_bits == 2)
                    Pcode = 'B';
                else if(two_bits == 3)
                    Pcode = 'U';

                
                if(PROTO == eld_dt_obd2)
                {
                    sprintf(next_node->spn, "%c%d%d%d%d", Pcode, nibbles[0]&0x03, nibbles[1], nibbles[2], nibbles[3]);
                    sprintf(next_node->eld_chars, "%c%d%d%d%d", Pcode, nibbles[0]&0x03, nibbles[1], nibbles[2], nibbles[3]);
                }
                else
                {
                    sprintf(next_node->spn, "%c%d%c%c%c", Pcode, nibbles[0]&0x03, *(data_buf_ptr+1), *(data_buf_ptr+2), *(data_buf_ptr+3));
                    sprintf(next_node->eld_chars, "%c%d%c%c%c", Pcode, nibbles[0]&0x03, *(data_buf_ptr+1), *(data_buf_ptr+2), *(data_buf_ptr+3));
                }

                data_buf_ptr+=4;

            break;

            case eld_dt_j1708:
                next_node->src = xtoi(DT_list_node->src,1);

                memset(next_node->spn, 0x00, sizeof(next_node->spn));
                strncpy(next_node->spn,data_buf_ptr, 6);

                memset(next_node->eld_chars, 0x00, sizeof(next_node->eld_chars));
                strncpy(next_node->eld_chars,data_buf_ptr, 6);  //6 chars
                data_buf_ptr+=6;
            break;

        }
        

        ptr1 = next_node;
        
    }

}
void format_ELD_buff(struct DT_info* info_list_HEAD, char* ELD_buff, uint8_t PROTO)
{
    if(ELD_buff == NULL)
        return;

    uint8_t nos_DTC = 0;

    char temp_bytes_buff[400] = {0x00};
    char bytes[5] = {0x00};

    memset(ELD_buff, 0x00, sizeof(ELD_buff));

    switch (PROTO)
    {
    case eld_dt_j1939:
        sprintf(ELD_buff, "$SDG&S=0&e=0,0,4,");
    break;
    
    case eld_dt_uds3:
    case eld_dt_obd2:
        sprintf(ELD_buff, "$SDG&S=0&e=0,0,1,");
    break;

    case eld_dt_j1708:
        sprintf(ELD_buff, "$SDG&S=0&e=0,0,2,");
    
    default:
        break;
    }
    

    while(info_list_HEAD!=NULL)
    {
        strcat(temp_bytes_buff,",");
        strcat(temp_bytes_buff, info_list_HEAD->eld_chars);

        nos_DTC++;
        info_list_HEAD = info_list_HEAD->next;

    }

    if(nos_DTC == 0)
    {
        memset(ELD_buff, 0x00, sizeof(ELD_buff));
        printf("\n\ryay, no DTC detected :)\n\r");
        return;
    }
    sprintf(bytes, "%d", nos_DTC);
    strcat(ELD_buff, bytes);
    strcat(ELD_buff, temp_bytes_buff);

    //printf("\n\r\n\rResponse to ELD: %s\n\r\n\r", ELD_buff);

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

int parse_byte_data(struct node_DT_raw* NODE, uint8_t* segment_buff, uint16_t len, uint8_t mFrameCnt, uint8_t PROTO)
{
    char buff[500];
    memset(buff, 0x00, sizeof(buff));
    strncpy(buff, segment_buff, len);
    
    char header_buff[12] = {0x00};  //just the header
    char hyphen_header[20] = {0x00}; //header + hyphen
    char hyph_fc_header[32] = {0x00}; //hyphen + header + frame counter
    char* ptr;
    
    uint16_t frame_offset_j1708 = 0; 
    // manually stat through the string to remove substrings
    

    memset(header_buff, 0x00, sizeof(header_buff));
    memset(hyphen_header, 0x00, sizeof(hyphen_header));


    switch(PROTO)
    {
        case eld_dt_j1939:
            ptr = strstr(buff, "FECA");
            if(ptr != NULL)
            {
                strncpy(header_buff , ptr-3, 9);
            }
            else if(strstr(buff, "FECB") != NULL)
            {
                ptr = strstr(buff, "FECB");
                strncpy(header_buff , ptr-3, 9);
            }
            else if(strstr(buff, "EBFF") != NULL)
            {
                ptr = strstr(buff, "EBFF");
                strncpy(header_buff , ptr-3, 9);
            }
            else
            {
                printf("\n\rno valid frame header found\n\r");
                return -1;
            }

            memset(NODE->src, 0x00, 5);
            NODE->src[0] = header_buff[7];
            NODE->src[1] = header_buff[8];
        break;

        case eld_dt_uds3:
        case eld_dt_uds_bb6:
            ptr = strstr(buff, "18DAF1");
            if(ptr != NULL)
            {
                strncpy(header_buff , ptr-1, 9);
            }
        break;

        case eld_dt_obd2:
            ptr = strstr(buff, "0:");

        case eld_dt_j1708:
            memset(NODE->src, 0x00, 5);
            strncpy(NODE->src, segment_buff+1, 4);
            break; //do nothing
            
    }
        
    //snprintf(hyphen_header,10,"-%s",header_buff);  //-Header

    

    if(mFrameCnt == 0)  //single frame header
    {
        switch (PROTO)
        {

        case eld_dt_j1939:
            removeSubstring(buff, header_buff);
        break;

        case eld_dt_uds3:
        case eld_dt_uds_bb6:
            memset(hyph_fc_header, 0x00, sizeof(hyph_fc_header));
            strncpy(hyph_fc_header, segment_buff, 15);
            removeSubstring(buff, hyph_fc_header);

        case eld_dt_obd2:
        //nothing to do
        removeSubstring(buff, "-");
        break;

        case eld_dt_j1708:
            memset(hyph_fc_header, 0x00, sizeof(hyph_fc_header));
            strncpy(hyph_fc_header, segment_buff, 7); // (-MID PID N)
            removeSubstring(buff, hyph_fc_header);
        break;
        
        default:
            break;
        }
        
    }
        

    else
    {
        for(int i = 1; i<=mFrameCnt; i++)  //multi frame header removal
        {
            memset(hyph_fc_header, 0x00, sizeof(hyph_fc_header));
            switch (PROTO)
            {
                case eld_dt_j1939:
                    sprintf(hyph_fc_header, "%s%.2X",header_buff,i);
                break;
                
                case eld_dt_uds3:
                case eld_dt_uds_bb6:
                    if(i == 1)  //response frame
                    {
                        strncpy(hyph_fc_header, segment_buff, 17);
                    }
                    else
                        sprintf(hyph_fc_header, "%s%.2X",header_buff,(0x20 + (i-1)));
                break;

                case eld_dt_obd2:
                    removeSubstring(buff, "-");
                    sprintf(hyph_fc_header, "%d:", (i-1));
                break;

                case eld_dt_j1708:
                    strncpy(hyph_fc_header, segment_buff+frame_offset_j1708, (i==1?13:11));
                    frame_offset_j1708 += (xtoi(hyph_fc_header+5,1)*2) + 7;  //(header + bytecnt + frame)
                    
                break;

                default:
                    break;
            }
            
            removeSubstring(buff, hyph_fc_header);
        }
    }

    switch (PROTO)  //echo removal
    {
        case eld_dt_j1939:
            //nothing to do
        break;

        case eld_dt_uds_bb6:
        case eld_dt_uds3:
        break;


        case eld_dt_obd2:
            ptr = strstr(buff, "43");  //03 srv
            if(ptr == NULL)
                ptr = strstr(buff, "47"); //07 srv

            memset(header_buff, 0x00, sizeof(header_buff));
            strncpy(header_buff, ptr, 4);
            removeSubstring(buff, header_buff);
        break;

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
uint8_t total_char_in_segment, uint16_t mFrameBytes, uint8_t mFrameCnt, uint8_t PROTO)
{
    struct node_DT_raw* node = (struct node_DT_raw*)malloc(sizeof(struct node_DT_raw));  
    
    struct node_DT_raw* ptr1 = *list_HEAD;

    if(ptr1 != NULL)
        while(ptr1->next != NULL)
            ptr1 = ptr1->next;

    parse_byte_data(node, segment_buff, total_char_in_segment, mFrameCnt, PROTO);
    
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
    char sid_1708[3] = {0x00};
    uint8_t multi_bytes_1708 = 0;
    uint8_t frame_cnt_1708 = 0;
    uint8_t chars_in_frame_1708;
    //leaf conditions (end recursion)

    switch(PROTO)
    {
        case eld_dt_j1939:
            if(len_main_buff<24)  
            {
                printf("\n\rreached end of DT, exiting");
                return 0;
            }
            

            if(strstr(ptr1,"FECA")==NULL &&  strstr(ptr1,"FECB")==NULL && strstr(ptr1,"ECFF")==NULL && strstr(ptr1,"EBFF")==NULL)  //no valid header found
            {
                printf("\n\rno valid header found, exiting");
                return -1;
            }
        break;

        case eld_dt_uds_bb6:
        case eld_dt_uds3:
            if(len_main_buff<24)  
            {
                printf("\n\rreached end of DT, exiting");
                return -1;
            }

            if(strstr(ptr1,"18DAF1")==NULL)  //no valid header found
            {
                printf("\n\rno valid header found, exiting");
                return -1;
            }
        break;

        case eld_dt_obd2:
            if(len_main_buff<14) // first frame of multiframe is 6 bytes + counter
            {
                printf("\n\rreached end of DT, exiting");
                return -1;
            }

            if(strstr(ptr1,"43")==NULL  &&  strstr(ptr1,"47")==NULL)  //03/07 echo
            {
                printf("\n\rno valid header found, exiting");
                return -1;
            }

        case eld_dt_j1708:
            if(len_main_buff<8)
            {
                printf("\n\r reached end of DT string, exiting");
                return -1;
            }

            /*if(strstr(ptr1,"-8C")==NULL  &&  strstr(ptr1,"-80")==NULL)  //discuss possible headers
            {
                printf("\n\r no valid header found");
                return -1;
            }*/

        break;

    }

    
    //parsing scheme. Chop the raw chars based on multiframe/singleframe and store in DT_raw_list
    switch(PROTO)
    {
        case eld_dt_j1939:
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
                total_char_in_segment = multi_frame_cnt*25;   //((multi_frame_bytes*2)+ (8*multi_frame_cnt)+(2*multi_frame_cnt) + multi_frame_cnt + (((multi_frame_cnt*7)-multi_frame_bytes))*2);
                
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

        case eld_dt_uds3:
        case eld_dt_uds_bb6: //Todo
            if(strstr(ptr1, "18DAF1") != NULL)
            {
                if(*ptr1 == '-')
                    ptr2 = ptr1+1;

                uint8_t len = xtoi(ptr2+8,1);

                if(xtoi(ptr2+8,1) == 0x10)  //multi frame header (byte 0)
                {
                    nos_multi_frame ++;
                    multi_frame_detected = 1;
                    multi_frame_bytes = xtoi(ptr2+10,1); //byte 1
                    multi_frame_cnt = multi_frame_bytes%7==0?(multi_frame_bytes/7):(multi_frame_bytes/7)+1;
                    printf("\n\rdetected multiframe nos:%d multiframe,byte cnt:%d , frame cnt = %d",nos_multi_frame,multi_frame_bytes, multi_frame_cnt);
                    total_char_in_segment = multi_frame_cnt*25;
                    ptr2 = ptr1;
                }

                else
                {
                    nos_single_frame++;
                    multi_frame_detected = 0;
                    printf("\n\rdetected single frame nos:%d",nos_single_frame);
                    ptr2 = ptr1; 
                    total_char_in_segment = 25;
                }
                    

            }
            else
            {
                printf("\n\r garbage data\n\r");
                break;
            } 
        break;


        case eld_dt_obd2:
            if(strstr(ptr1, "43") !=NULL  ||   strstr(ptr1, "47") !=NULL)
            {
                if(*(ptr1+2) == ':') //multiframe counter detected
                {
                    ptr2 = ptr1;
                    nos_multi_frame ++;
                    multi_frame_detected = 1;
                    for(int i =0; i<strlen(ptr2); ptr2++)
                    {
                        if(*ptr2 == ':')
                            multi_frame_cnt++;
                    }

                    multi_frame_bytes = (multi_frame_cnt*7) - 1; //first frame has 6 bytes 

                    if(*(ptr1+1) == '0')  //check if first frame 
                        total_char_in_segment = strlen(ptr1);  //for now, copy the whole buffer as there is no multi src

                    else
                        return -1; //something went wrong

                }

                else 
                {
                    nos_single_frame++;
                    multi_frame_detected = 0;
                    total_char_in_segment = strlen(ptr1);  //single frame
                } 
            }

            ptr2 = ptr1;

        break;


        case eld_dt_j1708:
            //multi frame
            
            memset(sid_1708, 0x00, sizeof(sid_1708));
            strncpy(sid_1708,ptr1+3,2);
            if(strncmp(sid_1708, "C0", 2) == 0)  {

                //if(*ptr1 == '-')
                    //ptr1++;

                nos_multi_frame ++;
                multi_frame_detected = 1;
                //multi_frame_cnt = ((xtoi(ptr1+9, 1) >> 4) & 0x0F)+1;  //upper nible +1
                
                multi_frame_bytes = xtoi(ptr1+11, 1);
                multi_bytes_1708 = multi_frame_bytes; //temp

                ptr2 = ptr1;
                while(multi_bytes_1708 != 0)
                {
                    chars_in_frame_1708 = (xtoi(ptr2+5,1)*2) + 7; //multi_frame_cnt==0?(xtoi(ptr2+5,1)*2) + 7:(xtoi(ptr2+5,1)*2) + 5;
                    multi_bytes_1708 += multi_frame_cnt==0?2:3; // add 2 or 3 extra bytes of actual SID, frame counter, data bytes (d) 
                    total_char_in_segment += chars_in_frame_1708;  //one frame
                    multi_bytes_1708 -= xtoi(ptr2+5,1);

                    ptr2 += chars_in_frame_1708;
                    multi_frame_cnt++;
                }

                ptr2 = ptr1;
                
            }
            //single frame
            else if(strncmp(sid_1708, "C2", 2) == 0)
            {
                //if(*ptr1 == '-')  //ignore hyphen
                    //ptr1++;

                nos_single_frame ++;
                multi_frame_detected = 0;

                total_char_in_segment = (xtoi(ptr1+5, 1)*2) + 7 ;
                ptr2 = ptr1;
            }

        break;

    }



    

    memset(segment_buff , 0x00, sizeof(segment_buff));
    strncpy(segment_buff ,ptr2, total_char_in_segment);

    if(PROTO != eld_dt_obd2)
    {
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
    }
    else if(PROTO == eld_dt_obd2)
        ptr2+=total_char_in_segment; 
    
    push_DT_segment_to_list(list_HEAD, segment_buff, total_char_in_segment, multi_frame_bytes , multi_frame_cnt, PROTO);
    //printf("total 1st seg = %d, total remaining = %d", total_char_in_segment, strlen(ptr2) );
    find_multi_src_frames(list_HEAD, ptr2 , strlen(ptr2), PROTO);

    return 0;
    
}