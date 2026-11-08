"""Small Migen simulation models; native-model equivalence is tested separately."""
from migen import Array, Cat, If, Instance, Module, Signal


class EfinixPrimitives:
    @staticmethod
    def lower(instance):
        ports = {item.name: item.expr for item in instance.items if hasattr(item, "expr")}
        params = {item.name: item.value for item in instance.items
                  if isinstance(item, Instance.Parameter)}
        model = Module()
        if instance.of == "EFX_SRL8":
            state = Signal(8, reset=params["INIT"].value, reset_less=True)
            address = Signal(3)
            model.comb += [address.eq(~ports["A"]), ports["Q"].eq(~Array(state)[address])]
            model.sync += If(ports["CE"], state.eq(Cat(ports["D"], state[:7])))
        elif instance.of == "EFX_DSP48":
            # The production timer uses C + the registered accumulator W.
            state = Signal(48, reset_less=True)
            model.comb += ports["O"].eq(state)
            model.sync += If(ports["RST"], state.eq(0)).Elif(
                ports["CE"], state.eq(state + ports["C"]))
        else:
            raise NotImplementedError(instance.of)
        return model


SPECIAL_OVERRIDES = {Instance: EfinixPrimitives}
