# PicoCart64

Lazy command for me:

```
cd .. && rm -rf build/* && ./scripts/load_rom.py --compress /home/carl/git/n64/PicoCart64/sw/n64/testrom/testrom.z64 && cd build/ && cmake -DREGION=NTSC -DFLASH_SIZE_MB=16  ..  && cmake --build . -j 20
```

Nintendo 64 flash cart using a Raspberry Pi RP2040

Join the Discord server to participate in the discussion and development of this project.

[![Join the Discord!](https://discordapp.com/api/guilds/989902502063398982/widget.png?style=banner3)](https://discord.gg/CGTjxkVr7P)
