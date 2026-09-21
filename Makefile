
GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/waveshare-roarm

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/waveshare-roarm.exe
endif

GO_SRC := $(shell find cmd internal components -name '*.go' 2>/dev/null)
EMBEDS := $(shell find internal/geometry -type f ! -name '*.go' 2>/dev/null)
$(if $(GO_SRC),,$(error GO_SRC is empty: has the source layout moved?))

$(MODULE_BINARY): Makefile go.mod $(GO_SRC) $(EMBEDS)
	$(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/module

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

vet:
	go vet ./...

test:
	go test -race ./...

module.tar.gz: meta.json $(MODULE_BINARY)
ifneq ($(VIAM_TARGET_OS), windows)
	strip $(MODULE_BINARY)
endif
	tar czf $@ meta.json $(MODULE_BINARY)

module: test module.tar.gz

all: test module.tar.gz

setup:
	go mod tidy
