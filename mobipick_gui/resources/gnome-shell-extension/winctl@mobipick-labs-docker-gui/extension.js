// Window enumeration and placement over D-Bus for GNOME Shell on Wayland.
//
// wmctrl and xprop only see X11 windows, so on a Wayland session the
// Mobipick Labs Docker GUI talks to this extension instead. Windows are
// addressed by the stable Meta.Window id (a uint64 rendered as a string).
import Gio from 'gi://Gio';
import Meta from 'gi://Meta';
import {Extension} from 'resource:///org/gnome/shell/extensions/extension.js';

const OBJECT_PATH = '/org/gnome/Shell/Extensions/MobipickWinCtl';
const IFACE = `
<node>
  <interface name="org.gnome.Shell.Extensions.MobipickWinCtl">
    <method name="ListWindows">
      <arg type="s" name="json" direction="out"/>
    </method>
    <method name="MoveResize">
      <arg type="s" name="id" direction="in"/>
      <arg type="i" name="x" direction="in"/>
      <arg type="i" name="y" direction="in"/>
      <arg type="i" name="width" direction="in"/>
      <arg type="i" name="height" direction="in"/>
      <arg type="b" name="ok" direction="out"/>
    </method>
    <method name="SetWorkspace">
      <arg type="s" name="id" direction="in"/>
      <arg type="i" name="index" direction="in"/>
      <arg type="b" name="ok" direction="out"/>
    </method>
    <method name="Activate">
      <arg type="s" name="id" direction="in"/>
      <arg type="b" name="ok" direction="out"/>
    </method>
    <method name="Unmaximize">
      <arg type="s" name="id" direction="in"/>
      <arg type="b" name="ok" direction="out"/>
    </method>
    <method name="SetAbove">
      <arg type="s" name="id" direction="in"/>
      <arg type="b" name="above" direction="in"/>
      <arg type="b" name="ok" direction="out"/>
    </method>
    <method name="Version">
      <arg type="i" name="version" direction="out"/>
    </method>
  </interface>
</node>`;

const PROTOCOL_VERSION = 2;

function isNormalWindow(win) {
    return win.get_window_type() === Meta.WindowType.NORMAL && !win.skip_taskbar;
}

function allWindows() {
    // Bottom-to-top stacking order across all workspaces.
    const actors = global.get_window_actors();
    const windows = actors.map(actor => actor.meta_window).filter(win => win);
    return global.display.sort_windows_by_stacking(windows);
}

function findWindow(id) {
    const wanted = String(id);
    return allWindows().find(win => String(win.get_id()) === wanted) ?? null;
}

export default class MobipickWinCtl extends Extension {
    enable() {
        this._dbus = Gio.DBusExportedObject.wrapJSObject(IFACE, this);
        this._dbus.export(Gio.DBus.session, OBJECT_PATH);
    }

    disable() {
        this._dbus?.unexport();
        this._dbus = null;
    }

    Version() {
        return PROTOCOL_VERSION;
    }

    ListWindows() {
        const entries = [];
        allWindows().forEach((win, stackIndex) => {
            if (!isNormalWindow(win))
                return;
            const rect = win.get_frame_rect();
            const workspace = win.get_workspace();
            entries.push({
                id: String(win.get_id()),
                title: win.get_title() ?? '',
                pid: win.get_pid() > 0 ? win.get_pid() : null,
                desktop: workspace ? workspace.index() : null,
                x: rect.x,
                y: rect.y,
                width: rect.width,
                height: rect.height,
                wm_class: [win.get_wm_class_instance(), win.get_wm_class()]
                    .filter(value => value),
                stack_index: stackIndex,
                maximized: win.get_maximized() !== 0,
                minimized: win.minimized,
                above: win.is_above(),
            });
        });
        return JSON.stringify(entries);
    }

    MoveResize(id, x, y, width, height) {
        const win = findWindow(id);
        if (!win)
            return false;
        if (win.get_maximized())
            win.unmaximize(Meta.MaximizeFlags.BOTH);
        if (win.minimized)
            win.unminimize();
        win.move_resize_frame(true, x, y, width, height);
        return true;
    }

    SetWorkspace(id, index) {
        const win = findWindow(id);
        if (!win)
            return false;
        const count = global.workspace_manager.get_n_workspaces();
        if (index < 0 || index >= count)
            return false;
        win.change_workspace_by_index(index, false);
        return true;
    }

    Activate(id) {
        const win = findWindow(id);
        if (!win)
            return false;
        win.activate(global.get_current_time());
        return true;
    }

    Unmaximize(id) {
        const win = findWindow(id);
        if (!win)
            return false;
        win.unmaximize(Meta.MaximizeFlags.BOTH);
        return true;
    }

    // Keep a window above the others. Wayland clients cannot request this
    // themselves, so Qt.WindowStaysOnTopHint is ignored on Wayland sessions.
    SetAbove(id, above) {
        const win = findWindow(id);
        if (!win)
            return false;
        if (above)
            win.make_above();
        else
            win.unmake_above();
        return true;
    }
}
