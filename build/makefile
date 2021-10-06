ifeq ($(OS),Windows_NT)
  $(info detected OS is Windows_NT)
  include makefileWin
else
  UNAME := $(shell uname)
  $(info detected OS uname is $(UNAME))
  include makefileLinux
endif
