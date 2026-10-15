import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() > 0:
  joystick = pygame.joystick.Joystick(0)
  joystick.init()
  print(f"Connected: {joystick.get_name()}")
else:
  print("No joystick detected.")

running = True
while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
    elif event.type == pygame.JOYBUTTONDOWN:
      print(f"Button pressed: {event.button}")
    elif event.type == pygame.JOYAXISMOTION:
      # L2 or R2 are often mapped to axes (e.g., axis 4 or 5)
      if abs(event.value) > 0.1:
        print(f"Axis {event.axis}: {event.value}")
