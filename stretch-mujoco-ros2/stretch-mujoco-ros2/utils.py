# import mujoco


# def get_actuator_names(mjmodel):
#     """
#     Get the names of all actuators in the model.

#     Args:
#         mjmodel (MjModel): The model to get the actuator names from.

#     Returns:
#         list: The names of all actuators in the model.
#         list[id] = actuator_name
#     """
#     num_actuators = mjmodel.nu  # Number of actuators
#     actuator_names = []

#     for i in range(num_actuators):
#         name = mujoco.mj_id2name(mjmodel, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
#         actuator_names.append(name)
#         print(f"Actuator {i}: {name}")
#     return actuator_names