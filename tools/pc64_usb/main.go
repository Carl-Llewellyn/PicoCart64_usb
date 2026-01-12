package main

import (
	"bufio"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"strings"
	"time"

	"golang.org/x/sys/unix"
)

const (
	packetSize = 22
	sync0      = 0x53
	sync1      = 0x4D
	version    = 1
)

type Packet struct {
	Header  uint32
	Version uint8
	Player  uint8
	F1      float32
	F2      float32
	F3      float32
	Buttons uint16
	StickX  int8
	StickY  int8
	Extra0  uint8
	Extra1  uint8
	Raw     [packetSize]byte
}

func decodePacket(buf []byte, wireBigEndian bool) (Packet, error) {
	var p Packet
	if len(buf) != packetSize {
		return p, fmt.Errorf("packet size %d, want %d", len(buf), packetSize)
	}
	copy(p.Raw[:], buf)
	p.Header = binary.BigEndian.Uint32(buf[0:4])
	p.Version = buf[2]
	p.Player = buf[3]
	if wireBigEndian {
		p.F1 = math.Float32frombits(binary.BigEndian.Uint32(buf[4:8]))
		p.F2 = math.Float32frombits(binary.BigEndian.Uint32(buf[8:12]))
		p.F3 = math.Float32frombits(binary.BigEndian.Uint32(buf[12:16]))
		p.Buttons = binary.BigEndian.Uint16(buf[16:18])
	} else {
		p.F1 = math.Float32frombits(binary.LittleEndian.Uint32(buf[4:8]))
		p.F2 = math.Float32frombits(binary.LittleEndian.Uint32(buf[8:12]))
		p.F3 = math.Float32frombits(binary.LittleEndian.Uint32(buf[12:16]))
		p.Buttons = binary.LittleEndian.Uint16(buf[16:18])
	}
	p.StickX = int8(buf[18])
	p.StickY = int8(buf[19])
	p.Extra0 = buf[20]
	p.Extra1 = buf[21]
	return p, nil
}

func encodePacket(header uint32, f1, f2, f3 float32, buttons uint16, stickX, stickY int8, extra0, extra1 uint8, wireBigEndian bool) [packetSize]byte {
	var buf [packetSize]byte
	binary.BigEndian.PutUint32(buf[0:4], header)
	if wireBigEndian {
		binary.BigEndian.PutUint32(buf[4:8], math.Float32bits(f1))
		binary.BigEndian.PutUint32(buf[8:12], math.Float32bits(f2))
		binary.BigEndian.PutUint32(buf[12:16], math.Float32bits(f3))
		binary.BigEndian.PutUint16(buf[16:18], buttons)
	} else {
		binary.LittleEndian.PutUint32(buf[4:8], math.Float32bits(f1))
		binary.LittleEndian.PutUint32(buf[8:12], math.Float32bits(f2))
		binary.LittleEndian.PutUint32(buf[12:16], math.Float32bits(f3))
		binary.LittleEndian.PutUint16(buf[16:18], buttons)
	}
	buf[18] = byte(stickX)
	buf[19] = byte(stickY)
	buf[20] = extra0
	buf[21] = extra1
	return buf
}

func openSerial(port string, baud int) (*os.File, error) {
	fd, err := unix.Open(port, unix.O_RDWR|unix.O_NOCTTY, 0666)
	if err != nil {
		return nil, err
	}

	if err := configureSerial(fd, baud); err != nil {
		_ = unix.Close(fd)
		return nil, err
	}

	return os.NewFile(uintptr(fd), port), nil
}

func configureSerial(fd int, baud int) error {
	termios, err := unix.IoctlGetTermios(fd, unix.TCGETS)
	if err != nil {
		return err
	}

	termios.Iflag &^= unix.IGNBRK | unix.BRKINT | unix.PARMRK | unix.ISTRIP | unix.INLCR | unix.IGNCR | unix.ICRNL | unix.IXON
	termios.Oflag &^= unix.OPOST
	termios.Lflag &^= unix.ECHO | unix.ECHONL | unix.ICANON | unix.ISIG | unix.IEXTEN
	termios.Cflag &^= unix.CSIZE | unix.PARENB
	termios.Cflag |= unix.CS8
	termios.Cc[unix.VMIN] = 1
	termios.Cc[unix.VTIME] = 0

	speed, err := baudRateConstant(baud)
	if err != nil {
		return err
	}
	termios.Cflag &^= unix.CBAUD
	termios.Cflag |= speed
	termios.Ispeed = speed
	termios.Ospeed = speed

	return unix.IoctlSetTermios(fd, unix.TCSETS, termios)
}

func baudRateConstant(baud int) (uint32, error) {
	switch baud {
	case 9600:
		return unix.B9600, nil
	case 19200:
		return unix.B19200, nil
	case 38400:
		return unix.B38400, nil
	case 57600:
		return unix.B57600, nil
	case 115200:
		return unix.B115200, nil
	case 230400:
		return unix.B230400, nil
	default:
		return 0, fmt.Errorf("unsupported baud rate %d", baud)
	}
}

func readPackets(r io.Reader, out io.Writer, wireBigEndian bool) error {
	var pktBuf [packetSize]byte
	idx := 0
	tmp := make([]byte, 64)
	for {
		n, err := r.Read(tmp)
		if err != nil {
			if errors.Is(err, io.EOF) {
				return nil
			}
			return err
		}
		if n == 0 {
			continue
		}
		for i := 0; i < n; i++ {
			b := tmp[i]
			if idx == 0 {
				if b != sync0 {
					continue
				}
				pktBuf[idx] = b
				idx = 1
				continue
			}
			if idx == 1 {
				if b != sync1 {
					idx = 0
					continue
				}
				pktBuf[idx] = b
				idx = 2
				continue
			}

			pktBuf[idx] = b
			idx++
			if idx >= packetSize {
				if pktBuf[0] == sync0 && pktBuf[1] == sync1 && pktBuf[2] == version {
					pkt, err := decodePacket(pktBuf[:], wireBigEndian)
					if err != nil {
						return err
					}
					fmt.Fprintf(out, "header=0x%08X version=%d player=%d f1=%f f2=%f f3=%f buttons=0x%04X stickX=%d stickY=%d extra0=0x%02X extra1=0x%02X raw=%s\n",
						pkt.Header,
						pkt.Version,
						pkt.Player,
						pkt.F1,
						pkt.F2,
						pkt.F3,
						pkt.Buttons,
						pkt.StickX,
						pkt.StickY,
						pkt.Extra0,
						pkt.Extra1,
						hex.EncodeToString(pkt.Raw[:]),
					)
				}
				idx = 0
			}
		}
	}
}

func parseHexPayload(input string) ([packetSize]byte, error) {
	var out [packetSize]byte
	clean := strings.NewReplacer(" ", "", "\n", "", "\t", "", ",", "", "0x", "", "0X", "").Replace(input)
	if len(clean)%2 != 0 {
		return out, fmt.Errorf("hex string must have even length")
	}
	decoded, err := hex.DecodeString(clean)
	if err != nil {
		return out, err
	}
	if len(decoded) != packetSize {
		return out, fmt.Errorf("hex payload must be %d bytes, got %d", packetSize, len(decoded))
	}
	copy(out[:], decoded)
	return out, nil
}

func sendPacket(w io.Writer, payload [packetSize]byte, repeat int, interval time.Duration) error {
	for i := 0; i < repeat; i++ {
		if _, err := w.Write(payload[:]); err != nil {
			return err
		}
		if i+1 < repeat && interval > 0 {
			time.Sleep(interval)
		}
	}
	return nil
}

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}

	switch os.Args[1] {
	case "read":
		readCmd := flag.NewFlagSet("read", flag.ExitOnError)
		port := readCmd.String("port", "/dev/ttyACM0", "serial device path")
		baud := readCmd.Int("baud", 115200, "serial baud rate (cdc ignores but required by tty)")
		wire := readCmd.String("wire", "be", "wire endianness for float/buttons: be or le")
		_ = readCmd.Parse(os.Args[2:])

		f, err := openSerial(*port, *baud)
		if err != nil {
			fatal(err)
		}
		defer f.Close()

		writer := bufio.NewWriter(os.Stdout)
		defer writer.Flush()

		wireBigEndian, err := parseWireEndian(*wire)
		if err != nil {
			fatal(err)
		}

		if err := readPackets(f, writer, wireBigEndian); err != nil {
			fatal(err)
		}
	case "send":
		sendCmd := flag.NewFlagSet("send", flag.ExitOnError)
		port := sendCmd.String("port", "/dev/ttyACM0", "serial device path")
		baud := sendCmd.Int("baud", 115200, "serial baud rate (cdc ignores but required by tty)")
		wire := sendCmd.String("wire", "be", "wire endianness for float/buttons: be or le")
		hexPayload := sendCmd.String("hex", "", "raw 22-byte payload as hex (overrides other fields)")
		header := sendCmd.Uint("header", 0, "header word (u32, sent big-endian)")
		f1 := sendCmd.Float64("f1", 0, "float32 #1")
		f2 := sendCmd.Float64("f2", 0, "float32 #2")
		f3 := sendCmd.Float64("f3", 0, "float32 #3")
		buttons := sendCmd.Uint("buttons", 0, "buttons (u16, sent little-endian)")
		stickX := sendCmd.Int("sx", 0, "stick X (int8)")
		stickY := sendCmd.Int("sy", 0, "stick Y (int8)")
		extra0 := sendCmd.Uint("extra0", 0, "byte 20 (u8)")
		extra1 := sendCmd.Uint("extra1", 0, "byte 21 (u8)")
		repeat := sendCmd.Int("repeat", 1, "number of times to send the packet")
		interval := sendCmd.Duration("interval", 0, "delay between repeats (e.g. 10ms)")
		_ = sendCmd.Parse(os.Args[2:])

		if *repeat < 1 {
			fatal(errors.New("repeat must be >= 1"))
		}
		if *buttons > 0xFFFF {
			fatal(errors.New("buttons out of range"))
		}
		if *stickX < -128 || *stickX > 127 || *stickY < -128 || *stickY > 127 {
			fatal(errors.New("stick values must be in -128..127"))
		}
		if *extra0 > 0xFF || *extra1 > 0xFF {
			fatal(errors.New("extra bytes must be in 0..255"))
		}

		wireBigEndian, err := parseWireEndian(*wire)
		if err != nil {
			fatal(err)
		}

		var payload [packetSize]byte
		if *hexPayload != "" {
			parsed, err := parseHexPayload(*hexPayload)
			if err != nil {
				fatal(err)
			}
			payload = parsed
		} else {
			payload = encodePacket(uint32(*header), float32(*f1), float32(*f2), float32(*f3), uint16(*buttons), int8(*stickX), int8(*stickY), uint8(*extra0), uint8(*extra1), wireBigEndian)
		}

		f, err := openSerial(*port, *baud)
		if err != nil {
			fatal(err)
		}
		defer f.Close()

		if err := sendPacket(f, payload, *repeat, *interval); err != nil {
			fatal(err)
		}
	default:
		usage()
		os.Exit(2)
	}
}

func usage() {
	fmt.Fprintf(os.Stderr, "usage: %s <read|send> [options]\n", os.Args[0])
}

func fatal(err error) {
	fmt.Fprintln(os.Stderr, "error:", err)
	os.Exit(1)
}

func parseWireEndian(value string) (bool, error) {
	switch strings.ToLower(value) {
	case "be", "big":
		return true, nil
	case "le", "little":
		return false, nil
	default:
		return false, fmt.Errorf("invalid wire endianness %q (use be or le)", value)
	}
}
