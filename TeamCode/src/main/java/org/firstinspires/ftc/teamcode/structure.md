yap yap yap

# subsystems

dt with sus
cata mech: barkertake + diff slides
cata xbow: pivot + release
shooter: turret + hood + flywheel
dye: 1m dye + (dolphins powered coax) + 1m transfer
intake: pivot (4b) + intake
vision: ll3a (turret) + arducam (back)

# actuators

see doc in discord

# subsystems

## superstructure stuff
cata transfer
intake folding off cata mode
cata mech autodeploy and stuff
cata xbow hijacking bot rotation for launch

passive triggers:
- pinpoint to detect when near cata, activates cata mode
- pinpoint to detect when near pipes, activates suspension mode

active triggers:
- on cata xbow launch bot rotation is hijacked

## shooter logic

basically the goal is massive so we dont rly care
constant based sotm compensation
cool ideas would be compensate by bot angle but it lowk doesn't matter
bang bang flywheel, lut

## cata mech logic

states for high/low/ground/transfer, automatic hook pull/hook dump routines when detected close enough.
no sensor :( so using current reads to detect whether we have a cata
transfer is a driver choice, similar to crescendo driving

## cata xbow logic

cata go wheee make up some logic here but it rly not that deep. 
xbow cannot pivot side to side so bot will autoalign for shot, 4144 style structured feeding

## dye logic

full speed all the time idk could run current limiting
transfer is fixed vel bang bang

## intake logic

folded state + out state, folds when cata is happening to increase bps

## suspension logic

triggers off a projected pose estimate from the pinpoint data

## vision logic

black box

# TODO

[X] dt
[ ] cata mech
[ ] cata xbow
[ ] shooter
[X] dye
[X] intake
[X] vision
[ ] superstructure



