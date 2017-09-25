import addon_utils, bpy


class ReloadMyself(bpy.types.Operator):
    bl_idname = "view3d.reload_myself"
    bl_label = "reload myself"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        print("123")
        addon_utils.disable("robot-arm-simulation")
        addon_utils.enable("robot-arm-simulation")
        return {"FINISHED"}

# handle the keymap
wm = bpy.context.window_manager
kc = wm.keyconfigs.addon
if kc.keymaps.get("3D View") == None:
    km = kc.keymaps.new('3D View')
else:
    km = kc.keymaps['3D View']

list = [it for it in km.keymap_items if it.idname==ReloadMyself.bl_idname]
if len(list)>0 :
    print( "km.keymap_items.remove()", list[0])
    km.keymap_items.remove(list[0])

bpy.utils.register_class(ReloadMyself)
km.keymap_items.new(ReloadMyself.bl_idname, 'R', 'PRESS', alt=True)
print("register reload operatior")

