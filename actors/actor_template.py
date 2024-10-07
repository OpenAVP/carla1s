class ActorTemplate:
    
    def __init__(self,
                 blueprint_name: str,
                 **attributes) -> None:
        self.blueprint_name = blueprint_name
        self.attributes = attributes
