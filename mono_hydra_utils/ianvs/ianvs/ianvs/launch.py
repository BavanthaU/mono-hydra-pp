import pathlib
import tempfile

from typing import Optional, Sequence, Text
from launch.launch_context import LaunchContext
from launch.frontend import expose_action, expose_substitution, Entity, Parser
from launch.frontend.parse_substitution import parse_substitution
from launch_ros.actions import Node
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions


class PyenvPrefix(Substitution):
    """Custom substitution for prefixing exec with correct interperter."""

    def __init__(
        self,
        pyenv: SomeSubstitutionsType,
        prefix: Optional[SomeSubstitutionsType] = None,
    ):
        super().__init__()
        self.pyenv = normalize_to_list_of_substitutions(pyenv)
        self.prefix = prefix
        if self.prefix is not None:
            self.prefix = normalize_to_list_of_substitutions(self.prefix)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        raise ValueError("Cannot directly parse!")

    def describe(self) -> Text:
        env_repr = " + ".join([s.describe() for s in self.pyenv])
        if self.prefix is not None:
            prefix_repr = ", pkg=" + " + ".join([s.describe() for s in self.prefix])
        else:
            prefix_repr = ""

        return f"PyenvExec(pyenv={env_repr}{prefix_repr})"

    def perform(self, context: LaunchContext) -> Text:
        pyenv = perform_substitutions(context, self.pyenv)
        if pyenv:
            pyinterp = pathlib.Path(pyenv) / "bin" / "python"
            if not pyinterp.exists():
                raise ValueError(f"Interperter '{pyinterp}' not found for '{pyenv}'")
        else:
            pyinterp = ""

        if self.prefix is None:
            return str(pyinterp)

        prefix = perform_substitutions(context, self.prefix)
        return f"{prefix} {pyinterp}"


@expose_action("pyenv_node")
class CustomNode(Node):
    """Custom node action that handles virtual environments."""

    def __init__(self, *, pyenv, prefix=None, **kwargs):
        """Set up the node."""
        super().__init__(prefix=PyenvPrefix(pyenv, prefix), **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        _, kwargs = super().parse(entity, parser)
        env = parser.parse_substitution(entity.get_attr("pyenv", optional=False))
        kwargs["pyenv"] = env
        return cls, kwargs


@expose_substitution("subs-file")
class RenderFileSubstitution(Substitution):
    """Custom substitution for performing substitutions on file contents."""

    def __init__(self, path: SomeSubstitutionsType):
        super().__init__()
        self.path = normalize_to_list_of_substitutions(path)
        self._new_file = None

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        if not data or len(data) != 1:
            raise AttributeError("Path required for file rendering substitution")

        return cls, {"path": data[0]}

    def describe(self) -> Text:
        path_repr = " + ".join([s.describe() for s in self.path])
        return f"RenderFile(path={path_repr})"

    def perform(self, context: LaunchContext) -> Text:
        path = perform_substitutions(context, self.path)
        path = pathlib.Path(path).expanduser().absolute()
        if not path.exists():
            raise SubstitutionFailure(f"File '{path}' does not exist!")

        with path.open("r") as fin:
            contents = fin.read()

        contents = parse_substitution(contents)
        contents = perform_substitutions(context, contents)
        with tempfile.NamedTemporaryFile(mode="w", delete=False) as fout:
            self._new_file = pathlib.Path(fout.name)
            fout.write(contents)

        return str(self._new_file)

    def __del__(self):
        if self._new_file:
            self._new_file.unlink()
