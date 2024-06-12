// Brake logic
void BrakeLogic(float Right,float Left)
{    
  // We brake when our input value is 0
  if (Right == 0.0 && Left == 0.0)
  {
    digitalWrite(rightBrake,LOW);
    digitalWrite(leftBrake,LOW);
  }
  else
  {
    digitalWrite(rightBrake,HIGH);
    digitalWrite(leftBrake,HIGH);
  }
}