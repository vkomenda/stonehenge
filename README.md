# Toy Stonehenge Mechanics Demo Using Rapier

## Prerequisites

* Nightly Rust

```bash
curl -s https://static.rust-lang.org/rustup.sh | sh -s -- --channel=nightly
```

See also the [book](https://doc.rust-lang.org/1.2.0/book/nightly-rust.html).

* Some dev library packages if requested during the build.

## Run

```bash
cargo run --release
```

Note that running in debug mode - that is, without `--release` - slows down the graphics
considerably.
