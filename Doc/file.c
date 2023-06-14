sensors_event_t mag_event;
sensors_vec_t   orientation;
  
/* Calculate the heading using the magnetometer */
mag.getEvent(&mag_event);
if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
{
  /* 'orientation' should have valid .heading data now */
  Serial.print(F("Heading: "));
  Serial.print(orientation.heading);
  Serial.print(F("; "));
}