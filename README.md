# SX127x_GS

## Installation

```
pip install git+https://github.com/CrinitusFeles/SX127x_GS.git
```

or

```
poetry add git+https://github.com/CrinitusFeles/SX127x_GS.git
```

## Using

```
python -m sx127x_gs COM17
```

After successfull initialization you can send data over LoRa by list of int or bytes.

```
> [253, 1, 233, 4]
```
or
```
> b'hello world!`
```
