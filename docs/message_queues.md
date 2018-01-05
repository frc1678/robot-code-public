# Message Queues

We pass around a lot of data through message queues. This page gives a simple example of how you would use a message queue - by reading through it, you should be able to get a pretty good idea of how message queues work.

You can create a message queue object, for example:

```C++
muan::queues::MessageQueue<std::string> example_queue(100);
```

That would be a queue of strings that can hold 100 strings at a time. You can write more than 100 messages to the queue, but once you go over 100 old messages will start to be deleted to make room for newer messages. In practice, messages getting deleted rarely matters.

Once you have a message queue object, you can write to it. Let's say that we write three messages:

```C++
example_queue.WriteMessage("Test message 1!");
example_queue.WriteMessage("Test message 2!");
example_queue.WriteMessage("Test message 3!");
```

Now, we can create an object to read messages:

```C++
auto example_queue_reader = example_queue.MakeReader();

std::cout << example_queue_reader.ReadMessage().value() << std::endl;
std::cout << example_queue_reader.ReadMessage().value() << std::endl;
std::cout << example_queue_reader.ReadMessage().value() << std::endl;
```

That would print out the following output:

```
Test message 1!
Test message 2!
Test message 3!
```

Note that this is in the same order that we wrote the messages in - the first thing that we put into the queue is the first thing that will come out of it. For this reason, message queues are sometimes called "FIFO queues" or "FIFO buffers", for "First-in, First-out".

Often times, we don't care about the old messages, we just want the most recent value. In these cases, we don't have to bother creating a queue reader, we can just call `ReadLastMessage()` on the queue:

```C++
std::cout << example_queue.ReadLastMessage().value() << std::endl;
```

Which would output:

```
Test message 3!
```

Note that we've been calling `.value()` on the messages that we get out of the queue. That's because the thing that we've been getting out of the queue isn't actually a `std::string`! The reason for that is that sometimes, there might not be any new messages in the queue - we need some way to represent that there's nothing new in the queue. For this purpose, the values returned from the message queues are "optionals" (`std::optional<std::string>`). You need to check if the optional has a value before you use it. One way to do this by using it as if it was a boolean:

```C++
auto example_output = example_queue.ReadLastMessage();

if (example_output) {
  std::cout << example_output.value() << std::endl;
} else {
  std::cout << "Nothing in the queue!" << std::endl;
}
```
