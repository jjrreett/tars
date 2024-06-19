extends Generic6DOFJoint3D

var body_a
var body_b
var target_position = Vector3(2, 1, 0)
var max_velocity = 2.0
var max_force = 100.0
var time = 0.0

func _ready():
	body_a = get_node(node_a) as RigidBody3D
	body_b = get_node(node_b) as RigidBody3D

func _physics_process(delta):
	time += delta
	var delta_pos = body_b.transform.origin - body_a.transform.origin
	var delta_vel = body_b.linear_velocity - body_a.linear_velocity
	
	# Update the target position every second to go up and down
	if int(time) % 2 == 0:
		target_position = Vector3(1, 1, 0)
	else:
		target_position = Vector3(1, -1, 0)
	
	target_position = Vector3(1, cos(time), sin(time))
		
	# Calculate the desired velocity to reach the target position
	var position_error = target_position - delta_pos
	var target_velocity = (10 * position_error).limit_length(max_velocity)
	var velocity_error = target_velocity - delta_vel
	
	# Calculate the force needed to achieve the desired velocity, clamped to the maximum force
	var force = (100 * (velocity_error * body_b.mass)).limit_length(max_force)
	
	print("position_error=", position_error, " force=", force)
	
	# Apply the force to both bodies
	body_a.apply_force(-force)
	body_b.apply_force(force)
