# Request to place text on the display.
# The display row and column as well as string specifics are required
# A comment field is used to assist user debug

# actionType values
uint8  DISPLAY_ALL=1              # Message to fill entire display
uint8  DISPLAY_SUBSTRING=2        # Message for a set of characters on one line
uint8  DISPLAY_STARTUP_STRING= 3  # Reserved for future use for non-volatile startup string
uint8  DISPLAY_SET_BRIGHTNESS= 4  # Sets display brightness. attributes is the brightness

int32  actionType                 # The type of action to be taken. See MSG_DISPLAY for values
int32  row                        # The display row of characters for this message
                                  # row can be from 0 to 7 where 0 is top line
int32  column                     # The starting horizontal pixel column
                                  # The column can be 0 - 118 where 0 is start at the left
int32  numChars                   # Number of characters to go to the display
int32  attributes                 # Used for brightness level 
string text                       # The text for this display update.  Up to 15 characters
string comment                    # Optional comment the user can use to document purpose
