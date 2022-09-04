def map(x, in_min, in_max, out_min, out_max): 
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def print_mapped_table():
  '''
    Prints a table of degrees representation in servo values.
    Only used for debugging.

    Returns: None
  '''
  print("\n\n")
  print("*********************************************************")
  print("|\tDegrees:\t|\tServo steering value:\t|")
  print("*********************************************************")
  for i in range(0, 46):
      x = map(i, -45, 45, -1, 1 )
      print(f"|\t{i}\t\t|\t{round(x, 2)}\t\t\t|")
  print("*********************************************************")
  print("\n\n")

def get_servo_value(servo_values, angle):
  '''Gets the correct value to steer the car.

    Parameters:
    --------------------
    servo_values (list) : List containing all values corresponding to a specific angle.
  '''
  if angle < -45 or angle > 45 or not isinstance(angle, int):
    print("Invalid format. Angle must be an integer in the range of -45 to + 45")
    return None

  if angle < 0:
    return -1*(servo_values[-1*angle])
  else:
    return servo_values[angle]


servo_values = [0.0, 0.02, 0.04, 0.07, 0.09, 0.11, 0.13, 0.16, 0.18, 0.2, 0.22, 0.24, 0.27, 0.29, 0.31, 0.33, 0.36, 0.38, 0.4, 0.42, 0.44, 0.47, 0.49, 0.51, 0.53, 0.56, 0.58, 0.6, 0.62, 0.64, 0.67, 0.69, 0.71, 0.73, 0.76, 0.78, 0.8, 0.82, 0.84, 0.87, 0.89, 0.91, 0.93, 0.96, 0.98, 1.0]

print_mapped_table()
#servo_value = get_servo_value(servo_values, 3)
#print(servo_value)

