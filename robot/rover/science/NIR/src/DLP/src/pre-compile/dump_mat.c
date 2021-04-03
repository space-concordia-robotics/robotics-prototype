#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdint.h"

/** Maximum Hadamard matrix present as binary file in the ./Hadamard_matrices directory */
#define MAX_HADAMARD_CREATED 1831

static void read_matrix_from_bin_file(FILE *fp, uint8_t *a, int n)
{
    int i,j;
    unsigned int bit_mask;
    unsigned int hadamard_word;
    int bit_index=0;

    //printf("Matrix =\n");

    for(i=0; i < n; i++)
    {
        for(j = 0; j < n; j++)
        {
            if(bit_index++ % 32 == 0)
            {
                fread(&hadamard_word, sizeof(unsigned int), 1, fp);
                bit_mask = 1<<31;
            }
            if(hadamard_word & bit_mask)
            {
                a[i*n+j] = 1;
            }
            else
                a[i*n+j] = 0;
            
            //printf("%d", a[i*n+j]);

            bit_mask >>= 1;
        }
        //printf("\n");
    }
}

int dump_h_file(int order, FILE *output_fp)
{
    uint8_t *s_matrix;
    int i,j;
    uint8_t temp;
    FILE *hadamard_fp;
    char hadamard_filename[2048];
    //char output_filename[2048];

    if(output_fp == NULL)
        return -3;
    
    sprintf(hadamard_filename, "Hadamard_matrices/s_mat_%d.bin", order);
    hadamard_fp = fopen(hadamard_filename, "r");
    if(hadamard_fp == NULL)
    {
       //DEBUG_ERR("Unable to open %s\n", hadamard_filename);
       return -2;
    }

    s_matrix = malloc(sizeof(uint8_t)*((order*order+7)/8 * 8));
    memset(s_matrix, 0, sizeof(uint8_t)*((order*order+7)/8 * 8));

    if(s_matrix == NULL)
    {
        //DEBUG_ERR("Memory allocation for hadamard matrices failed\n");
        return -1;
    }

    read_matrix_from_bin_file(hadamard_fp, s_matrix, order);

    fprintf(output_fp, "const uint8_t g_s_matrix_%u[%u] = {", order, (order*order+7)/8);
    for(i=0; i < order * order; i+=8)
    {
        temp = 0;
        //printf("Hadamard binary: ");
        for(j=0; j < 8; j++)
        {
            //printf("%hhu ", s_matrix[i+j]);
            temp = temp + (int)(s_matrix[i+j] << j);
        }
        //printf("compressed as: %hhu\n", temp);
        fprintf(output_fp, "%hhu", temp);
        
        if(i < (order*order-1))
            fprintf(output_fp, ",");
    }
    fprintf(output_fp, "};\n");
    
    fclose(hadamard_fp);
    fclose(output_fp);
    
    free(s_matrix);
    
    return 0;
}

int checkPrime(int n)
{
    int i,isPrime=1;
    for(i=2;i<=n/2;++i)
    {
        if(n%i==0)
        {
            isPrime=0;
            break;
        }
    }

    return isPrime;
}

int getPaleyOrder(int n)
{
    n++;
    if(n%4!=0)
        n += (4-(n%4));

    while(!checkPrime(n-1))
        n += 4;

    return n-1;
}

int parseDoubleMatrix(uint8_t *packedMatrix, double *unpackedMatrix, int order)
{
    int i,j;
    //uint8_t mask;
    double temp;
       
    for(i=0; i<(order*order+7)/8; i++)
    {
//        printf("packed[%u] = %u\n", i, packedMatrix[i]);
        for(j=0; j<8; j++)
        {
            //mask = 1 << j;
            temp = (double)((packedMatrix[i] >> j) & 1);
//            printf("unpacked[%u] = %f\n", i*8+j, temp);
            unpackedMatrix[i*8+j] = temp;
        }
    }
    return 0;
}

int main(int argc, char *argv[])
/**
 * Processes the binary pre-computed Hadamard matrices in ./Hadamard_matrices
 * into packed uint8_t arrays for efficient storage on an embedded system.
 *
 * Re-exporting these header files is only necessary when you wish to change the
 * maximum Hadamard matrix size supported as defined in #HAD_MATRIX_MAX_ORDER_REQ.
 * When changing this value, you should compile this command line utility and run 
 * it to generate the necessary dlpspec_had_mat_#.h header files.
 */
{
    
    int i,j, order;
    FILE *output_fp;
    char output_filename[2048];
    double *unpackedMatrix;
    char *assign_string[4096];
        
    for(i=1;i<256;i++)
    {
//        printf("i = %i\n", i);
//        printf("paley = %i\n", getPaleyOrder(i));
        if(getPaleyOrder(i) == i)
        {
            sprintf(output_filename, "dlpspec_had_mat_%d.h", i);
            output_fp = fopen(output_filename, "w+");
            if(output_fp == NULL)
            {
                printf("Unable to open %s\n", output_filename);
                return -1;
            }
            
//            printf("order i = %i", i);
            order = dump_h_file(i, output_fp);
//            printf("Return: %i\n", i);
        }
    }
    
    sprintf(output_filename, "../dlpspec_had_defs.h");
    output_fp = fopen(output_filename, "w+");
    
    fprintf(output_fp, "#define MAX_HADAMARD_CREATED %u\n\n", MAX_HADAMARD_CREATED);
    fprintf(output_fp, "#if HAD_MATRIX_MAX_ORDER_REQ > MAX_HADAMARD_CREATED\n");
    fprintf(output_fp, "#error Required matrices not found\n");
    fprintf(output_fp, "#endif\n\n");
    
    for(i=MAX_HADAMARD_CREATED; i>0; i--)
    {
//        printf("check = %u\n", i);
//        printf("order = %u\n", getPaleyOrder(i));
        if(getPaleyOrder(i) == i)
        {
            fprintf(output_fp, "#if HAD_MATRIX_MAX_ORDER_REQ >= %u\n", i);
            fprintf(output_fp, "#include \"pre-compile/dlpspec_had_mat_%u.h\"\n", i);
            fprintf(output_fp, "#endif\n\n");
        }
    }
    
    for(i=0; i<MAX_HADAMARD_CREATED+1; i++)
    {
        if(getPaleyOrder(i) == i)
        {
            fprintf(output_fp, "#if HAD_MATRIX_MAX_ORDER_REQ >= %u && HAD_MATRIX_MAX_ORDER_REQ < %u\n", i, getPaleyOrder(i+1));
            fprintf(output_fp, "#define HAD_MATRIX_MAX_ORDER_AVAIL %u\n", i);
            fprintf(output_fp, "const uint8_t *g_matrix_lookup[%u] = {", i+1);
            for(j=0; j<=i; j++)
            {
                if(getPaleyOrder(j) == j)
                    fprintf(output_fp, "g_s_matrix_%u", j);
                else
                    fprintf(output_fp, "NULL");
                if(j<i)
                    fprintf(output_fp, ", ");
            }
            fprintf(output_fp, "};\n");
            fprintf(output_fp, "#endif\n\n");
        }
    }
    fclose(output_fp);
    
}
