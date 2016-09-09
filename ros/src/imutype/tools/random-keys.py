#!/usr/bin/env python3
import time, random
import tty, sys, termios

KEYS = list('TYUGHJBNM') + ['space']
MIN_DELAY = 2
MAX_DELAY = 2

# KEYS = [
#     'T', 'Y', 'U', 'I', 'O', 'P', '[', ']', 'return',
#     'G', 'H', 'J', 'K', 'L', ';', "'", '\\',
#     'V', 'B', 'N', 'M', ',', '.', '/',
#     'space',
# ]
# MIN_DELAY = 1.5
# MAX_DELAY = 2.5

keymap = {
    13: 'return',
    32: 'space',
    39: "'",
    44: ',',
    46: '.',
    47: '/',
    59: ';',
    91: '[',
    92: '\\',
    93: ']',
    98: 'B',
    103: 'G',
    104: 'H',
    105: 'I',
    106: 'J',
    107: 'K',
    108: 'L',
    109: 'M',
    110: 'N',
    111: 'O',
    112: 'P',
    116: 'T',
    117: 'U',
    118: 'V',
    121: 'Y',
}


counts = dict()

fmt = ' '.join(['{:>' + str(max(3, len(k) + 1)) + '}' for k in KEYS])

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

iteration = 0
while True:
    iteration += 1

    maxcount = max(counts.values() or [0]) - 10
    possible = [k for k in KEYS if counts.get(k, 0) < maxcount] or KEYS
    key = random.choice(possible)

    print(key + ' -> ', end='')
    sys.stdout.flush()

    start = time.time()
    pressed = getch()
    end = time.time()

    if ord(pressed) == 27:
        print('')
        break

    pressed_key = keymap[ord(pressed)] if ord(pressed) in keymap else 'OTHER'
    print(str(pressed_key), end='')
    sys.stdout.flush()
    counts[pressed_key] = (counts[pressed_key] if pressed_key in counts else 0) + 1


    interval = random.uniform(MIN_DELAY, MAX_DELAY)
    wait = interval - (end - start)

    if pressed_key != key:
        print(' (Wrong key)', end='')
        sys.stdout.flush()

    if wait > 0:
        time.sleep(wait)
        print('')
    else:
        print(' (too slow by {:.1f} seconds)'.format(-wait))

    if iteration % 10 == 0:
        print('=' * 79)
        print('Iteration {}'.format(iteration))
        print(fmt.format(*KEYS))
        print(fmt.format(*[counts[k] if k in counts else 0 for k in KEYS]))
        print('=' * 79)
