#include "HC_SR04.h"

//HC_SR04 *HC_SR04::_instance=NULL;
HC_SR04 *HC_SR04::_instance(NULL);

HC_SR04::HC_SR04(int trigger, int echo, int interrupt, int max_dist)
    : _trigger(trigger), _echo(echo), _int(interrupt), _max(max_dist), _finished(false)
{
  if(_instance==0) _instance=this;    
}

void HC_SR04::begin(){
  pinMode(_trigger, OUTPUT);
  digitalWrite(_trigger, LOW);
  pinMode(_echo, INPUT);  

}

void HC_SR04::start(){
  _finished=false;
  digitalWrite(_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger, LOW);  
}

unsigned int HC_SR04::getRange(bool units){
  return (_end-_start)/((units)?58:148);
}
