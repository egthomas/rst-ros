/* msgmem.h
   ========
   Author: R.J.Barnes
*/

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
 
 
 
*/

int writeraw(char *buf,int sze);
void readsock(fd_set *fdset,char *tmp_buf,int tmp_sze);
int writesock();
