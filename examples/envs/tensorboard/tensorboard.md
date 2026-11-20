\page bitrl_example_tensorboard Logging to Tensorboard

An experiment is easier to follow when its numbers are plotted as it runs.
\ref bitrl::network::TensorboardServer "TensorboardServer" sends values to a small server that
writes them for <a href="https://www.tensorflow.org/tensorboard">Tensorboard</a>, with PyTorch's
`SummaryWriter`. This example reads the settings of an experiment from `config.json` and logs some
of them.

## What you need

- a Tensorboard server that `TensorboardServer` can reach at `http://0.0.0.0:8002`, with PyTorch
  installed on its machine;
- `config.json`, which the build copies next to the executable. Set `log_path` in it to where the
  server should write the logs.

## What it logs

`TensorboardServer` has `add_scalar`, `add_scalars` and `add_text`. The example creates the
directory of the experiment from `experiment_dict` and `experiment_id`, logs the learning rate,
the seed and the number of epochs as scalars, and the optimizer as text.

## Running the example

@code{.sh}
./tensorboard
tensorboard --logdir <log_path>
@endcode

## The code

\include tensorboard.cpp
