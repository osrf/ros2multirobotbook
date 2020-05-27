![github pages](https://github.com/codebot/ros2multirobotbook/workflows/github%20pages/badge.svg)

# ros2multirobotbook

Greetings. Welcome to `ros2multirobotbook`. We hope you enjoy your stay.

The book is automatically compiled by changes to this repo. The compiled book
can be accessed here:

https://osrf.github.io/ros2multirobotbook/

# Compiling the book locally

It's convenient to compile the book locally when making edits. The following
steps will help you install the necessary tools.

### Dependencies

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

### Compiling the book

```
mdbook build
```
Then you can view the output:
```
firefox book/index.html
```

Alternatively, `mdbuild` can automatically trigger a rebuild on edits to
any source file:
```
mdbook watch
```
