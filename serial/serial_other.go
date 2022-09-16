// Copyright 2018 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

//go:build !freebsd && !linux && !netbsd && !solaris && !darwin && !dragonfly && !openbsd
// +build !freebsd,!linux,!netbsd,!solaris,!darwin,!dragonfly,!openbsd

package serial

var acceptedBauds map[uint32]uint32
