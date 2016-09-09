# CPU (4 Threads)
export THEANO_FLAGS=openmp=True
export OMP_NUM_THREADS=4
python script.py

# GPU
export THEANO_FLAGS=device=cuda,floatX=float32
python script.py
