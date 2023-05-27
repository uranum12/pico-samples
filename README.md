# pico samples

## build

```sh
cmake -G Ninja -B build
cmake --build build
```

## flash

```sh
sudo picotool load -v -x build/<filename>.uf2
```
