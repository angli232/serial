// +build darwin dragonfly freebsd netbsd openbsd

package serial

import (
	"errors"
	"unsafe"

	"golang.org/x/sys/unix"
)

const ioctlReadTermios = unix.TIOCGETA
const ioctlWriteTermios = unix.TIOCSETA

func (p *Port) flush() error {
	const FREAD = 0x01
	const FWRITE = 0x02
	com := int(FREAD | FWRITE)
	_, _, e := unix.Syscall6(unix.SYS_IOCTL,
		p.f.Fd(),
		unix.TCIOFLUSH,
		uintptr(unsafe.Pointer(&com)),
		0, 0, 0)
	if e != 0 {
		return e
	}
	return nil
}

func (t *termios) GetBaudrate() (baudrate int, err error) {
	if t == nil {
		return 0, errors.New("nil pointer for termios")
	}
	if t.Ispeed != t.Ospeed && t.Ispeed != unix.B0 {
		return 0, errors.New("Different input and output baudrate.")
	}
	switch t.Ospeed {
	case unix.B0:
		baudrate = 0
	case unix.B50:
		baudrate = 50
	case unix.B75:
		baudrate = 75
	case unix.B110:
		baudrate = 110
	case unix.B134:
		baudrate = 134
	case unix.B150:
		baudrate = 150
	case unix.B200:
		baudrate = 200
	case unix.B300:
		baudrate = 300
	case unix.B600:
		baudrate = 600
	case unix.B1200:
		baudrate = 1200
	case unix.B1800:
		baudrate = 1800
	case unix.B2400:
		baudrate = 2400
	case unix.B4800:
		baudrate = 4800
	case unix.B9600:
		baudrate = 9600
	case unix.B19200:
		baudrate = 19200
	case unix.B38400:
		baudrate = 38400
	case unix.B57600:
		baudrate = 57600
	case unix.B115200:
		baudrate = 115200
	case unix.B230400:
		baudrate = 230400
	default:
		return 0, ErrUnexpected
	}
	return baudrate, nil
}

func (t *termios) SetBaudrate(baudrate int) error {
	if t == nil {
		return errors.New("nil pointer for termios")
	}
	switch baudrate {
	case 0:
		t.Ispeed = unix.B0
		t.Ospeed = unix.B0
	case 50:
		t.Ispeed = unix.B50
		t.Ospeed = unix.B50
	case 75:
		t.Ispeed = unix.B75
		t.Ospeed = unix.B75
	case 110:
		t.Ispeed = unix.B110
		t.Ospeed = unix.B110
	case 134:
		t.Ispeed = unix.B134
		t.Ospeed = unix.B134
	case 150:
		t.Ispeed = unix.B150
		t.Ospeed = unix.B150
	case 200:
		t.Ispeed = unix.B200
		t.Ospeed = unix.B200
	case 300:
		t.Ispeed = unix.B300
		t.Ospeed = unix.B300
	case 600:
		t.Ispeed = unix.B600
		t.Ospeed = unix.B600
	case 1200:
		t.Ispeed = unix.B1200
		t.Ospeed = unix.B1200
	case 1800:
		t.Ispeed = unix.B1800
		t.Ospeed = unix.B1800
	case 2400:
		t.Ispeed = unix.B2400
		t.Ospeed = unix.B2400
	case 4800:
		t.Ispeed = unix.B4800
		t.Ospeed = unix.B4800
	case 9600:
		t.Ispeed = unix.B9600
		t.Ospeed = unix.B9600
	case 19200:
		t.Ispeed = unix.B19200
		t.Ospeed = unix.B19200
	case 38400:
		t.Ispeed = unix.B38400
		t.Ospeed = unix.B38400
	case 57600:
		t.Ispeed = unix.B57600
		t.Ospeed = unix.B57600
	case 115200:
		t.Ispeed = unix.B115200
		t.Ospeed = unix.B115200
	case 230400:
		t.Ispeed = unix.B230400
		t.Ospeed = unix.B230400
	default:
		return ErrInvalidParam
	}
	return nil
}
