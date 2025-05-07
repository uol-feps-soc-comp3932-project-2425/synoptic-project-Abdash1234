# event_manager.py

class EventManager:
    # An event manager that allows the registration of event callbacks and triggers
    def __init__(self):
        self.listeners = {}

    def register(self, event_name, callback):

        # Register a callback for a given event.

        if event_name not in self.listeners:
            self.listeners[event_name] = []
        self.listeners[event_name].append(callback)

    def trigger(self, event_name, *args, **kwargs):

        # Triggers all callbacks associated with an event
        if event_name in self.listeners:
            for callback in self.listeners[event_name]:
                callback(*args, **kwargs)

    
