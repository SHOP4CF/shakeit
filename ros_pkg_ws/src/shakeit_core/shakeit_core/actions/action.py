
class Action:

    def __init__(self):
        self.name = ''

    def perform_action(self) -> bool:
        raise NotImplementedError("Must be implemented in sub-classes")

    def __str__(self) -> str:
        return self.name
