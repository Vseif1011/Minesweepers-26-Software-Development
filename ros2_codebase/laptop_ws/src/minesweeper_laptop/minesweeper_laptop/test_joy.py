
import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected: {joystick.get_name()}")
print(f"Axes: {joystick.get_numaxes()}")
print(f"Buttons: {joystick.get_numbuttons()}")
print(f"Hats: {joystick.get_numhats()}")

running = True

try:
    while running:
        for event in pygame.event.get():

            if event.type == pygame.JOYAXISMOTION:
                print(
                    f"Axis {event.axis}: {event.value:.3f}"
                )

            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} PRESSED")

            elif event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} RELEASED")

            elif event.type == pygame.JOYHATMOTION:
                print(f"Hat {event.hat}: {event.value}")

            elif event.type == pygame.QUIT:
                running = False

except KeyboardInterrupt:
    pass

finally:
    pygame.quit()
