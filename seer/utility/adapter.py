from abc import ABC, abstractclassmethod
from typing import Generic, TypeVar


FromT = TypeVar("FromT")
ToT = TypeVar("ToT")


class Adapter(ABC, Generic[FromT, ToT]):
    @abstractclassmethod
    def convert(cls, to_convert: FromT) -> ToT:
        pass
