#!/bin/bash
#export MDBOOK_BOOK__title="programming_multiple_robots_with_ros_2"
export MDBOOK_OUTPUT__LATEX__latex=true
export MDBOOK_OUTPUT__LATEX__pdf=true
export MDBOOK_OUTPUT__LATEX__markdown=true
export RUST_BACKTRACE=full
mdbook build
