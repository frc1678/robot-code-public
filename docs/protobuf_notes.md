# Protobuf Notes

[Protobuf](https://developers.google.com/protocol-buffers/) is a format for storing data that was created by Google. We use protobuf to encode the data that we send between threads. You can take a look at what one of our protobufs looks like [here](../c2017/subsystems/superstructure/shooter/shooter.proto).

There are a few special things about the way that we use protobufs, which aren't clear from Google's documentation.

## Arenas

We use ["arena allocation"](https://en.wikipedia.org/wiki/Arena_allocation) for protobufs. This is needed for creating a protobuf object to be realtime. Because of this, all of our protobufs have `option cc_enable_arenas = true;` near the top.

## Timestamp

The first field on all of our protos is a timestamp, so that we can log what time the proto was sent at. We have [code](../muan/queues/message_queue.hpp) that automatically fills out this field, but you must manually put it in your protos. Do this by making the first field in all of your messages `required uint64 timestamp = 1;`.
