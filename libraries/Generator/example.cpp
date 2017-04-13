#include <stdio.h>
#include <iostream>
#include "RF24generator.h"


using namespace std;

int main(void)
{ 
  char word[4] = {'S','I','S','I'};
  RF24_adresses generator;
  bool b=generator.generate(word);
 
  cout << generator.printGenerator()<<endl;
  return 0;
}
