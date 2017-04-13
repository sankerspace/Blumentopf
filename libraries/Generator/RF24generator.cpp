#include "RF24generator.h"


RF24_adresses::RF24_adresses(void):position(0)
{
  grouplist=LinkedList<char**>();
}


RF24_adresses::~RF24_adresses(void)
{
  grouplist.~LinkedList();
}

int RF24_adresses::adressExists(char word[4])
{
  char** tmp;
  for(int i=0;i<grouplist.size();i++){
    tmp=grouplist.get(i); //only the first array is enough
    if(tmp[0][0]==word[0]){//all arrays are the same
      if(tmp[0][1]==word[1]){
        if(tmp[0][2]==word[2]){
          if(tmp[0][3]==word[3]){
          cout<<"size grouplist: "<<grouplist.size()<<endl;
            return i;// this group adress already in use
          }
        }        
      }
    }
  
  }
 
  return -1;
}

bool RF24_adresses::generate(char word[4])
{
  
  char** group =  new char*[5];
  //check if that address is already available
  
  
  if(adressExists(word)>0)
    return false;
////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  for(int i=0;i<5;i++)
  {
     group[i][0]=word[0];
     group[i][1]=word[1];
     group[i][2]=word[2];
     group[i][3]=word[3];
  }
  group[0][4]=(char)BASIS_1;
  group[1][4]=(char)BASIS_2;
  group[2][4]=(char)BASIS_3;
  group[3][4]=(char)BASIS_4;
  group[4][4]=(char)BASIS_5;
  
  
 
  
  grouplist.add(group);
  cout<<"size grouplist: "<<grouplist.size()<<endl;
  return true;
}


  void RF24_adresses::reset(void)
  {
    position=0;
  }
  
  bool RF24_adresses::increase(void)
  {
   position++;
   if(position <= grouplist.size())
    { //always a position which is new
      return true; 
    }
    return false;
  }
  
  char** RF24_adresses::nextGroup(void)
  {

    if(increase())
      return grouplist.get(position);
    else
      return (char**)(0);
    
  }
  
  
  bool  RF24_adresses::Group_delete(int i)
  {
     int s=grouplist.size();
     grouplist.remove(i);
     if(s>grouplist.size())
      return true;
      
     return false;
  }
  
  bool  RF24_adresses::Group_delete(char word[4])
  {
    int i = adressExists(word);
    if(i>=0)
      return Group_delete(i);
      
    return false;
  }
  
  
  
  char* RF24_adresses::printGenerator(void)
  {
    int len=grouplist.size()*25 + grouplist.size() + 10; //length of every table, every newline
    int pos=0;
    char* text = new char[len];
    char** group=0;
  //check if that address is already available
  
    group=nextGroup();
    
    while(group!=(char**)0)
    {
       for(int i=0;i<5;i++)
      {
        text[pos++]=group[i][0];
        text[pos++]=group[i][1];
        text[pos++]=group[i][2];
        text[pos++]=group[i][3];
        text[pos++]=group[i][4];
        text[pos++]='\n';
        
      }
        text[pos++]='\n'; 
      
      group=nextGroup();
    }
    
    reset();
 
    text[len-1]='\0';
    return text;
  }






