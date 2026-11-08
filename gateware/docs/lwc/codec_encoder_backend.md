# `codec_encoder_backend`

## Role

`codec_encoder_backend` composes the complete coefficient-domain encoder:
quantizer, per-band pair queues, and causal group encoder. It is the integration
point for a system that already provides the twelve raw transform-event lanes.

## Interface

Inputs are active-high reset, synchronous `clear_frame`, twelve raw valid bits,
and twelve signed fourteen-bit coefficient values. Outputs are `out_sof`,
`out_valid`, one sixteen-bit word, and aggregate `protocol_error`.

The wrapper has no input ready and no output ready. It connects the queue and
group encoder with ready/valid internally and ties the group encoder's
`out_ready` high.

## Dataflow and timing

The quantizer is combinational. Pair queues absorb the transform's bursty
multi-band events and replay complete groups. The group encoder may deassert
its pair ready while finalizing or serializing; the queues retain the current
pair and continue buffering ingress within their fixed capacity.

`out_sof` is valid only with the first `out_valid` word. Output may continue
after coefficient ingress has stopped, because queued groups, packet tokens,
and fixed-frame padding must drain.

## Error behavior and constraints

The externally observable error is the OR of queue capacity/protocol failure
and group-encoder failure. Quantizer coloring errors are asserted in simulation
but are not included in the synthesized aggregate because the production
transform schedule statically guarantees the conflict graph. The wrapper's
assertions additionally check all internal errors, group count ordering, a
maximum committed-minus-serialized backlog of 257, and exactly 64,000 groups
at `packets_done`.
