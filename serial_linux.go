package serial

import (
	"errors"

	"golang.org/x/sys/unix"
)

const ioctlReadTermios = unix.TCGETS
const ioctlWriteTermios = unix.TCSETS

func (p *Port) flush() error {
	_, _, e := unix.Syscall6(unix.SYS_IOCTL,
		p.f.Fd(),
		unix.TCFLSH,
		unix.TCIOFLUSH,
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
	cbaud := t.Cflag & (unix.CBAUD | unix.CBAUDEX)
	switch cbaud {
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
	var cbaud uint32
	switch baudrate {
	case 0:
		cbaud = unix.B0
	case 50:
		cbaud = unix.B50
	case 75:
		cbaud = unix.B75
	case 110:
		cbaud = unix.B110
	case 134:
		cbaud = unix.B134
	case 150:
		cbaud = unix.B150
	case 200:
		cbaud = unix.B200
	case 300:
		cbaud = unix.B300
	case 600:
		cbaud = unix.B600
	case 1200:
		cbaud = unix.B1200
	case 1800:
		cbaud = unix.B1800
	case 2400:
		cbaud = unix.B2400
	case 4800:
		cbaud = unix.B4800
	case 9600:
		cbaud = unix.B9600
	case 19200:
		cbaud = unix.B19200
	case 38400:
		cbaud = unix.B38400
	case 57600:
		cbaud = unix.B57600
	case 115200:
		cbaud = unix.B115200
	case 230400:
		cbaud = unix.B230400
	default:
		return ErrInvalidParam
	}
	t.Cflag &^= unix.CBAUD | unix.CBAUDEX
	t.Cflag |= cbaud
	return nil
}
