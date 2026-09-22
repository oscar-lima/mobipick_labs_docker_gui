// Window enumeration and placement over D-Bus for GNOME Shell on Wayland.
//
// wmctrl and xprop only see X11 windows, so on a Wayland session the
// Mobipick Labs Docker GUI talks to this extension instead. Windows are
// addressed by the stable Meta.Window id (a uint64 rendered as a string).
import Gio from 'gi://Gio';
import Meta from 'gi://Meta';
import Shell from 'gi://Shell';
import St from 'gi://St';
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
    <method name="ClearAttention">
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
    <method name="SetAppGlow">
      <arg type="s" name="app_id" direction="in"/>
      <arg type="d" name="level" direction="in"/>
      <arg type="s" name="color" direction="in"/>
      <arg type="i" name="count" direction="out"/>
    </method>
    <method name="Version">
      <arg type="i" name="version" direction="out"/>
    </method>
  </interface>
</node>`;

const PROTOCOL_VERSION = 4;

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

// Dock, dash and app-grid icons for a desktop id. GNOME 45+ ignores
// _NET_WM_ICON, so a window cannot change its own launcher icon; the shell
// side has to style the icon actors instead. Every AppIcon (and the
// dash-to-dock subclasses Ubuntu Dock uses) carries `.app` and an
// `.icon.icon` St.Icon texture, so the stage is walked for those.
function collectAppIcons(appId, node, out) {
    if (node.app?.get_id?.() === appId && node.icon?.icon instanceof St.Icon)
        out.push(node);
    if (typeof node.get_children === 'function')
        node.get_children().forEach(child => collectAppIcons(appId, child, out));
}

function isLiveActor(actor) {
    // A dock that rebuilt its icons leaves disposed GObjects behind; GJS
    // throws on any access to those, so treat that as "gone" too.
    try {
        return Boolean(actor) && !actor.is_finalized?.() && actor.get_stage() !== null;
    } catch (_error) {
        return false;
    }
}

function glowStyle(level, color) {
    const blur = Math.round(4 + 14 * level);
    const spread = Math.round(1 + 5 * level);
    return `icon-shadow: ${color} 0 0 ${blur}px ${spread}px;`;
}

export default class MobipickWinCtl extends Extension {
    enable() {
        this._glowCache = new Map();
        // The glow is state inside the shell, not inside the application
        // that asked for it: a GUI that is killed (or hangs and gets
        // force-quit) never sends the level-0 call, and the halo would
        // stay on the dock until the shell restarts.  Drop it ourselves
        // when the app's last window closes.
        this._appStateId = Shell.AppSystem.get_default().connect(
            'app-state-changed', (_system, app) => this._onAppStateChanged(app));
        this._dbus = Gio.DBusExportedObject.wrapJSObject(IFACE, this);
        this._dbus.export(Gio.DBus.session, OBJECT_PATH);
    }

    disable() {
        if (this._appStateId) {
            Shell.AppSystem.get_default().disconnect(this._appStateId);
            this._appStateId = null;
        }
        for (const appId of [...this._glowCache.keys()])
            this.SetAppGlow(appId, 0, '');
        this._glowCache = null;
        this._dbus?.unexport();
        this._dbus = null;
    }

    _onAppStateChanged(app) {
        if (!this._glowCache || app.state !== Shell.AppState.STOPPED)
            return;
        const appId = app.get_id();
        if (this._glowCache.has(appId))
            this.SetAppGlow(appId, 0, '');
    }

    _appIcons(appId) {
        let entries = this._glowCache.get(appId);
        if (!entries || entries.some(entry => !isLiveActor(entry.actor))) {
            const found = [];
            collectAppIcons(appId, global.stage, found);
            const known = new Map((entries ?? []).map(entry => [entry.actor, entry]));
            entries = found.map(actor => known.get(actor) ?? {
                actor,
                previous: actor.icon.icon.get_style(),
            });
            this._glowCache.set(appId, entries);
        }
        return entries;
    }

    // Surround every launcher icon of `app_id` (a desktop id such as
    // "foo.desktop") with a glow. `level` runs from 0 (restore the original
    // style) to 1 (strongest); `color` is a CSS colour such as
    // "rgba(120,200,255,0.9)".
    SetAppGlow(appId, level, color) {
        if (!this._glowCache)
            return 0;
        const entries = this._appIcons(appId);
        const strength = Math.max(0, Math.min(1, Number(level) || 0));
        let count = 0;
        for (const entry of entries) {
            if (!isLiveActor(entry.actor))
                continue;
            const texture = entry.actor.icon.icon;
            if (strength <= 0)
                texture.set_style(entry.previous);
            else
                texture.set_style(`${entry.previous ? `${entry.previous}; ` : ''}${glowStyle(strength, color)}`);
            count += 1;
        }
        if (strength <= 0)
            this._glowCache.delete(appId);
        return count;
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

    ClearAttention(id) {
        const win = findWindow(id);
        if (!win)
            return false;
        win.unset_demands_attention();
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
