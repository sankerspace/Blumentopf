#ifndef _RF24_GENERATOR_H_
#define _RF24_GENERATOR_H_

#include "LinkedList.h"
#include <iostream>

#define BASIS_1 0xC2  //pipe1
#define BASIS_2 0xC3  //pipe2
#define BASIS_3 0xC4  //pipe3
#define BASIS_4 0xC5  //pipe4
#define BASIS_5 0xC6  //pipe5

using namespace std;


class RF24_adresses
{
  private:
    int position;
    LinkedList<char**> grouplist;
    int adressExists(char word[4]);
    bool increase(void);
  public:
    RF24_adresses(void); 
    ~RF24_adresses(void);  
    bool generate(char word[4]);
   
    
    void reset(void);
    char** nextGroup(void);
   
    bool Group_delete(int i);
    bool Group_delete(char word[4]);
    
    char* printGenerator(void);//returns text
    
};


#endif
