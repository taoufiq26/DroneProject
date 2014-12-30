void setup(){
  Serial.begin(115200);
}
void loop(){
 int s=80;
if(s<=0)
  s=0;
  Serial.print(s);
}
