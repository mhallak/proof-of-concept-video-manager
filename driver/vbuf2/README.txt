Version 1.1.18

Changes audio devices from separate cards to channels on the same card

Eg:
"arecord -l":

card 0: TW6869 [TW6869], device 0: TW68 PCM [TW68 PCM]
  Subdevices: 8/8
  Subdevice #0: vch0 audio
  Subdevice #1: vch1 audio
  Subdevice #2: vch2 audio
  Subdevice #3: vch3 audio
  Subdevice #4: vch4 audio
  Subdevice #5: vch5 audio
  Subdevice #6: vch6 audio
  Subdevice #7: vch7 audio

Loopback on channel 2 (zero indexed):

"arecord -f S16_LE -r 48000 -D hw:TW6869,0,2 | aplay"


Version 1.1.17

Fixes 48KHz audio problem

