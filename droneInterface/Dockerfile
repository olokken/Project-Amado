FROM golang:1.21 AS builder

ENV GO111MODULE=on \
    CGO_ENABLED=0 \
    GOOS=linux \
    GOARCH=amd64

WORKDIR /app

COPY go.mod go.sum ./

RUN go mod download

COPY . .

RUN go build -o main ./cmd/droneInterface/main.go

FROM alpine:latest

ENV TZ=UTC

COPY --from=builder /app/main /

ENTRYPOINT ["/main"]

EXPOSE 8080
