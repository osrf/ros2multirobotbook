# ros2book

# compiling the book

### dependencies

Install a few dependencies, then Rust and `cargo`, its package manager:
```
sudo apt-get install libfontconfig1-dev libgraphite2-dev libharfbuzz-dev libicu-dev libssl-dev zlib1g-dev
sudo apt install cargo
```

Append cargo to the end of your `~/.bashrc` file:
```
export PATH=$PATH:$HOME/.cargo/bin
```

Now use `cargo` to install `mdbook`:
```
. ~/.bashrc
cargo install mdbook
cargo install mdbook-latex
cargo install tectonic
cargo install mdbook-epub
```

### compiling the book
