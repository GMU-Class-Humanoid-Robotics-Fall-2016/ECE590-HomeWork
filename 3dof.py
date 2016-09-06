import sys
import math

def main():
	if len(sys.argv) == 4:
		degree_one = float(sys.argv[1])
		degree_two = float(sys.argv[2])
		degree_three = float(sys.argv[3])
		
	else:
		print 'Usage: 3dof.py x, y, z'
		return
	
	position = calculate_position(degree_one, degree_two, degree_three)
	print(position)
	
def calculate_position(x, y, z):
	X_position = 0
	Y_position = 0

	# define the lengths of the robot arms
	l_one = 4
	l_two = 3
	l_three = 6
	
	# calculate the non-relative degree angles
	t1_degrees = x
	t2_degrees = x + y
	t3_degrees = x + y + z
	
	# convert to radians
	theta1 = math.radians(t1_degrees)
	theta2 = math.radians(t2_degrees)
	theta3 = math.radians(t3_degrees)

	angles = [theta1, theta2, theta3]
	lengths = [l_one, l_two, l_three]
	
	# calculate the position of the end effector
	for i in range (0,3):
		X_position += lengths[i] * math.cos(angles[i])
		Y_position += lengths[i] * math.sin(angles[i])
	
	X_position = float("{0:.2f}".format(X_position))
	Y_position = float("{0:.2f}".format(Y_position))
	
	position = (X_position, Y_position)
	return position
	
if __name__ == '__main__':
	main()
