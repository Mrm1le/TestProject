#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

int main(){

        int *data, size, count, i;

        printf( "fyi: your ints are %d bytes large\n", sizeof(int) );

        printf( "Enter number of ints to malloc: " );
        scanf( "%d", &size );
        data = (int *)malloc( sizeof(int) * size );
        if( !data ){
                perror( "failed to malloc" );
                exit( EXIT_FAILURE );
        }

        printf( "Enter number of ints to initialize: " );
        scanf( "%d", &count );
        for( i = 0; i < count; i++ ){
                data[i] = 1337;
        }

        printf( "I'm going to hang out here until you hit <enter>" );
        while( getchar() != '\n' );
        while( getchar() != '\n' );

        exit( EXIT_SUCCESS );
}