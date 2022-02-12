#!/usr/bin/env python

import math, pygame


'''
    Things to change:

        Color shingles
            based on the roof state
        update takes in a roof_state message


'''



class TileRenderer:
    def __init__(self, screen_width, screen_height, roof_height, roof_width, tile_buffer, roof_margin):
        self.tile_buffer = tile_buffer

        self.roof_margin = roof_margin
        self.screen_width = screen_width
        
        self.num_tiles_wide = roof_width
        self.num_tiles_high = roof_height
        print(f"desired roof width in tiles {self.num_tiles_wide}")
        print(f"desired roof height in tiles {self.num_tiles_high}")

        self.tile_width_px = int((screen_width - 2 *roof_margin) / (self.num_tiles_wide - .5))
        self.tile_height_px = int(screen_height / self.num_tiles_high)

        print(f"Number of tiles wide: {self.num_tiles_wide}")
        print(f"Number of tiles high: {self.num_tiles_high}")

        print(f"Tile width px: {self.tile_width_px}")
        print(f"Tile height px: {self.tile_height_px}")

    def getTileRect(self, x, y):
        #       x0 y0 w  h
        rect = [0, 0, 0, self.tile_height_px]

        rect[0] = x * (self.tile_width_px + self.tile_buffer) + self.roof_margin
        rect[1] = y * (self.tile_height_px + self.tile_buffer)

        # If we are on an odd row, offset x
        if y % 2 == 1:
            if x == 0:
                rect[2] = int(self.tile_width_px / 2)
                rect[0] = max(rect[0] - int(self.tile_width_px / 2), 0) + self.roof_margin
            elif x == self.num_tiles_wide:
                rect[2] = int(self.tile_width_px / 2)
                rect[0] = max(rect[0] - int(self.tile_width_px / 2), 0)
                
            else:
                rect[2] = self.tile_width_px

                rect[0] = max(rect[0] - int(self.tile_width_px / 2), 0) 
        else:
            if x == self.num_tiles_wide - 1:
                rect[2] = int(self.tile_width_px / 2)
            else:
                rect[2] = self.tile_width_px

        return rect

    def draw_inchworm(self, screen, id, ee_bot_pos, ee_bot_status, ee_top_pos, ee_top_status):
        # make this draw a line instead with some way to know id and top vs bot
        text_size = self.tile_height_px
        font = pygame.freetype.SysFont("arial", text_size/4)
        font.rotation = 0
        ee_bot_x = ee_bot_pos[0] * self.tile_width_px + self.tile_width_px/2
        ee_bot_y = ee_bot_pos[1] * self.tile_height_px + self.tile_height_px/2
        
        ee_top_x = ee_top_pos[0] * self.tile_width_px + self.tile_width_px/2
        ee_top_y = ee_top_pos[1] * self.tile_height_px + self.tile_height_px/2

        pygame.draw.line(screen, (100, 100, 100), (ee_bot_x, ee_bot_y), (ee_top_x, ee_top_y), width=int(self.tile_height_px/8))

        color = (0, 255, 0, 255)
        if ee_bot_status == 1:
            color = (255, 0, 0, 255)
        font.render_to(screen, (ee_bot_x, ee_bot_y), str(id) + "B", fgcolor=color)
        

        color = (0, 255, 0, 255)
        if ee_top_status == 1:
            color = (255, 0, 0, 255)
        font.rotation = 180

        font.render_to(screen, (ee_top_x, ee_top_y), "T" + str(id) , fgcolor=color)


    def drawRoof(self, screen, roof_state, shingle_depots_pos, inchworms):
        # draw shingle states
        print(len(roof_state))
        for row in range(self.num_tiles_high):
            for col in range(self.num_tiles_wide):
                rect = self.getTileRect(col, row)
                color = (0, 0, 0)
                # print(row * self.num_tiles_wide + col)
                if roof_state[row * self.num_tiles_wide + col] == 1:
                    color = (0, 0, 150)
                elif roof_state[row * self.num_tiles_wide + col] == 2:
                    color = (0, 0, 255)
                pygame.draw.rect(screen, color, rect)

        # draw shingle depots
        pygame.draw.circle(screen, (0, 255, 0), (self.roof_margin/2, shingle_depots_pos[0] * self.tile_height_px + self.tile_height_px/2), self.roof_margin/2.5)
        if len(shingle_depots_pos) > 1:
            pygame.draw.circle(screen, (0, 255, 255), (self.screen_width - self.roof_margin/2, shingle_depots_pos[1] * self.tile_height_px + self.tile_height_px/2), self.roof_margin/2.5)

        # draw inchworms
        print(len(inchworms))
        for i, worm in enumerate(inchworms):
            print(f"drawing inchwomr {i}")
            inchworm_id = worm.id
            ee1_pos = worm.ee1_pos
            ee1_status = worm.ee1_status
            ee2_pos = worm.ee2_pos
            ee2_status = worm.ee2_status
            self.draw_inchworm(screen, inchworm_id, ee1_pos, ee1_status, ee2_pos, ee2_status)


