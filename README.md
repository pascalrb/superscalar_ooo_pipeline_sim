# Superscalar OOO Pipeline Simulator (Dynamic Instruction Scheduling)

This simulator implements an out-of-order superscalar processor that fetches and issues N instructions per cycle. Only the dynamic scheduling mechanism is modeled in detail, i.e., perfect caches and perfect branch prediction are assumed.

Guided by a project from ECE563 at NC State University taught by Prof. [Eric Rotenberg](https://ece.ncsu.edu/people/ericro/).


# Instructions on Using This Simulator
1. Type "make" to build.  (Type "make clean" first if you already compiled and want to recompile from scratch.)

2. Run trace reader:
   To run without throttling output:
   ```
   ./sim 256 32 4 gcc_trace.txt
   ./sim 256 32 4 traces/gcc_trace.txt
   ./sim 256 32 4 traces/val_trace_gcc1 | less
   ```

   To run with throttling (via "less"):
   ```
   ./sim 256 32 4 traces/gcc_trace.txt | less
   ```
