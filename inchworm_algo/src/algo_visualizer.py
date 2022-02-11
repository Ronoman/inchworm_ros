#!/usr/bin/env python3

import rospy, sys, std_msgs, pygame, math

from inchworm_algo.msg import ShingleMsg, RoofState

from tile_renderer import TileRenderer



def handle_new_roof_state(msg): # will probobly have to pass in whatever pygame uses
    print("handling new RoofState")
    roof_state = list(msg.shingles)
    shingle_depot = list(msg.shingle_depot)




if __name__ == '__main__':
    # TODO: pass in a param to allow you to viz the map of a given inchworm
    rospy.init_node("algo_visualizer")



    WIDTH = 1500
    HEIGHT = 1000
    FPS = 60

    # Define Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)

    ## initialize pygame and create window
    pygame.init()
    pygame.mixer.init()  ## For sound
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Hex Rendering")
    clock = pygame.time.Clock()     ## For syncing the FPS

    ### Rendering constants ###
    TILE_BUFFER = 1 # px between drawn tiles
    ###########################

    ### Project constants ###
    ROOF_WIDTH  = 5 # in
    ROOF_HEIGHT = 10 # in

    TILE_WIDTH  = 12
    TILE_HEIGHT = 12
    #########################

    MARGINS = 50

    tile_renderer = TileRenderer(WIDTH , HEIGHT , ROOF_HEIGHT, ROOF_WIDTH, TILE_BUFFER, MARGINS)

    roof_state = [1] * WIDTH * HEIGHT
    shingle_depot_pos = [0, 5]


    rospy.Subscriber('/algo/roof_state', RoofState, handle_new_roof_state)
    print(f" roof size {len(roof_state)}")
    running = True
    while not rospy.is_shutdown() and running:
        #1 Process input/events
        clock.tick(FPS)     ## will make the loop run at the same speed all the time
        for event in pygame.event.get():        # gets all the events which have occured till now and keeps tab of them.
            ## listening for the the X button at the top
            if event.type == pygame.QUIT:
                running = False

        #3 Draw/render
        screen.fill(WHITE)

        tile_renderer.drawRoof(screen, roof_state, shingle_depot_pos)
        screen.blit(pygame.transform.flip(screen, False, True), (0, 0))

        ## Done after drawing everything to the screen
        pygame.display.flip()
    pygame.quit()
