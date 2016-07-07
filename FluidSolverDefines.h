#pragma once

#define IX(i,j) ((i%N)+(N+2)*(j%N))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) { if (solid[((i)+(N+2)*(j))]==0) {
#define END_FOR }}}