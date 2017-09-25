import bpy

def console_get():
    for area in bpy.context.screen.areas:
        if area.type == 'CONSOLE':
            for space in area.spaces:
                if space.type == 'CONSOLE':
                    return area, space
    return None, None


def output(*args):
    area, space = console_get()
    if space is None:
        return

    context = bpy.context.copy()
    context.update(dict(
        space=space,
        area=area,
    ))

    text = ""

    for v in args:
        if type(v) is str:
            text += v + "  "
        else:
            text += str(v) + "  "

    for line in text.split("\n"):
        bpy.ops.console.scrollback_append(context, text=line, type='OUTPUT')