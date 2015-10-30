import sys

def menu(title, menu):
    print(title)
    for i in range(1,len(menu)+1) :
        print('{} - {}'.format(i,menu[i-1]))
    continueLoop = True
    while continueLoop :
        error = False
        answer = raw_input('Please type your answer (or \'q\' to quit) : ')
        try:
            val = int(answer)
        except:
            if answer == 'q' or answer == 'Q':
                sys.exit("Exit required by user")
            else:
                print('Invalid answer, please type a number.')
                error = True
        if error == False:
            if val > 0 and val <= len(menu):
                continueLoop = False
            else:
                print('"{}" is not a valid menu entry. Availables options are :'.format(val))
                for i in range(1,len(menu)+1) :
                    print('{} - {}'.format(i,menu[i-1]))
    return val
    
# !!!! The following function is from StackOverflow.com !

# update_progress() : Displays or updates a console progress bar
## Accepts a float between 0 and 1. Any int will be converted to a float.
## A value under 0 represents a 'halt'.
## A value at 1 or bigger represents 100%
def update_progress(progress):
    barLength = 50 # Modify this to change the length of the progress bar
    status = ""
    if isinstance(progress, int):
        progress = float(progress)
    if not isinstance(progress, float):
        progress = 0
        status = "error: progress var must be float\r\n"
    if progress < 0:
        progress = 0
        status = "Halt...\r\n"
    if progress >= 1:
        progress = 1
        status = "Done...\r\n"
    block = int(round(barLength*progress))
    text = "\rPercent: [{0}] {1}% {2}".format( "#"*block + "-"*(barLength-block), progress*100, status)
    sys.stdout.write(text)
    sys.stdout.flush()